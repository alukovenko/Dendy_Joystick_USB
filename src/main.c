#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>
#include <string.h>

// --- Pin Definitions ---
// Matches the GPIO Mapping Table
// Port A
#define BUTTON_UP_PIN GPIO0
#define BUTTON_DOWN_PIN GPIO1
#define BUTTON_LEFT_PIN GPIO2
#define BUTTON_RIGHT_PIN GPIO3
#define BUTTON_SELECT_PIN GPIO4
#define BUTTON_START_PIN GPIO5
#define BUTTON_A_PIN GPIO6
#define BUTTON_B_PIN GPIO7
// Port B
#define BUTTON_TURBO_A_PIN GPIO0
#define BUTTON_TURBO_B_PIN GPIO1

// --- Button Bitmask Definitions for HID Report ---
#define BUTTON_UP_MASK (1 << 0)
#define BUTTON_DOWN_MASK (1 << 1)
#define BUTTON_LEFT_MASK (1 << 2)
#define BUTTON_RIGHT_MASK (1 << 3)
#define BUTTON_SELECT_MASK (1 << 4)
#define BUTTON_START_MASK (1 << 5)
#define BUTTON_A_MASK (1 << 6)
#define BUTTON_B_MASK (1 << 7)
#define BUTTON_TURBO_A_MASK (1 << 8)
#define BUTTON_TURBO_B_MASK (1 << 9)

// --- Timing Constants ---
#define DEBOUNCE_MS 20
#define TURBO_PERIOD_MS 33 // ~15 Hz (1000ms / 15Hz / 2)

// --- Global State Variables (volatile as they are modified in ISR) ---
volatile uint32_t system_millis = 0;
volatile uint16_t debounced_state = 0;
volatile bool turbo_toggle_state = false;

// --- USB HID Report Descriptor ---
static const uint8_t hid_report_descriptor = {
    0x05, 0x01, // Usage Page (Generic Desktop)
    0x09, 0x05, // Usage (Game Pad)
    0xA1, 0x01, // Collection (Application)
    0x05, 0x09, //   Usage Page (Button)
    0x19, 0x01, //   Usage Minimum (Button 1)
    0x29, 0x10, //   Usage Maximum (Button 16)
    0x15, 0x00, //   Logical Minimum (0)
    0x25, 0x01, //   Logical Maximum (1)
    0x75, 0x01, //   Report Size (1)
    0x95, 0x10, //   Report Count (16)
    0x81, 0x02, //   Input (Data,Var,Abs)
    0xC0,       // End Collection
};

// --- USB Descriptors ---
static const struct usb_device_descriptor dev_descr = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = 0,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x1209,  // pid.codes VID
    .idProduct = 0x0001, // Placeholder PID
    .bcdDevice = 0x0100,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

static const struct usb_endpoint_descriptor hid_endpoint = {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x81, // IN endpoint 1
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 2, // 16 bits for buttons
    .bInterval = 10,     // Poll every 10ms
};

static const struct usb_hid_descriptor hid_func_descriptor = {
    .bLength = sizeof(struct usb_hid_descriptor),
    .bDescriptorType = USB_DT_HID,
    .bcdHID = 0x0111,
    .bCountryCode = 0,
    .bNumDescriptors = 1,
    .bDescriptorType0 = USB_DT_REPORT,
    .wDescriptorLength0 = sizeof(hid_report_descriptor),
};

static const struct usb_interface_descriptor hid_iface = {
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 1,
    .bInterfaceClass = USB_CLASS_HID,
    .bInterfaceSubClass = 0, // No boot interface
    .bInterfaceProtocol = 0, // None
    .iInterface = 0,
    .endpoint = &hid_endpoint,
    .extra = &hid_func_descriptor,
    .extralen = sizeof(hid_func_descriptor),
};

static const struct usb_interface interfaces = {hid_iface};

static const struct usb_config_descriptor config = {
    .bLength = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType = USB_DT_CONFIGURATION,
    .wTotalLength = 0, // Will be filled by the driver
    .bNumInterfaces = 1,
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = 0x80, // Bus-powered
    .bMaxPower = 0x32,    // 100mA
    .interface = interfaces,
};

static const char *usb_strings = {
    "OpenSourceHardware",
    "NES Advantage USB",
    "12345",
};

static uint8_t usbd_control_buffer;
usbd_device *usbd_dev;

// --- USB Callbacks ---
static int hid_control_request(usbd_device *dev, struct usb_setup_data *req, uint8_t **buf,
                               uint16_t *len, void (**complete)(usbd_device *dev, struct usb_setup_data *req))
{
    (void)complete;
    (void)dev;

    if ((req->bmRequestType & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_STANDARD)
    {
        if (req->bRequest == USB_REQ_GET_DESCRIPTOR)
        {
            if ((req->wValue >> 8) == USB_DT_REPORT)
            {
                *buf = (uint8_t *)hid_report_descriptor;
                *len = sizeof(hid_report_descriptor);
                return 1;
            }
        }
    }
    return 0;
}

static void hid_set_config(usbd_device *dev, uint16_t wValue)
{
    (void)wValue;
    usbd_ep_setup(dev, 0x81, USB_ENDPOINT_ATTR_INTERRUPT, 2, NULL);
    usbd_register_control_callback(
        dev,
        USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
        USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
        hid_control_request);
}

// --- System Setup ---
static void clock_setup(void)
{
    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
}

static void gpio_setup(void)
{
    // Configure Port A pins as input with pull-up
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN,
                  BUTTON_UP_PIN | BUTTON_DOWN_PIN | BUTTON_LEFT_PIN | BUTTON_RIGHT_PIN |
                      BUTTON_SELECT_PIN | BUTTON_START_PIN | BUTTON_A_PIN | BUTTON_B_PIN);
    gpio_set(GPIOA, BUTTON_UP_PIN | BUTTON_DOWN_PIN | BUTTON_LEFT_PIN | BUTTON_RIGHT_PIN |
                        BUTTON_SELECT_PIN | BUTTON_START_PIN | BUTTON_A_PIN | BUTTON_B_PIN);

    // Configure Port B pins as input with pull-up
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN,
                  BUTTON_TURBO_A_PIN | BUTTON_TURBO_B_PIN);
    gpio_set(GPIOB, BUTTON_TURBO_A_PIN | BUTTON_TURBO_B_PIN);
}

static void systick_setup(void)
{
    systick_set_reload(rcc_ahb_frequency / 1000 - 1);
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_interrupt_enable();
    systick_counter_enable();
}

// --- Interrupt Service Routine ---
void sys_tick_handler(void)
{
    system_millis++;

    // --- Debouncing Logic ---
    static uint8_t counters[10] = {0};
    static uint16_t raw_state = 0;
    uint16_t current_raw_state = 0;

    // Read all pins
    if (!gpio_get(GPIOA, BUTTON_UP_PIN))
        current_raw_state |= BUTTON_UP_MASK;
    if (!gpio_get(GPIOA, BUTTON_DOWN_PIN))
        current_raw_state |= BUTTON_DOWN_MASK;
    if (!gpio_get(GPIOA, BUTTON_LEFT_PIN))
        current_raw_state |= BUTTON_LEFT_MASK;
    if (!gpio_get(GPIOA, BUTTON_RIGHT_PIN))
        current_raw_state |= BUTTON_RIGHT_MASK;
    if (!gpio_get(GPIOA, BUTTON_SELECT_PIN))
        current_raw_state |= BUTTON_SELECT_MASK;
    if (!gpio_get(GPIOA, BUTTON_START_PIN))
        current_raw_state |= BUTTON_START_MASK;
    if (!gpio_get(GPIOA, BUTTON_A_PIN))
        current_raw_state |= BUTTON_A_MASK;
    if (!gpio_get(GPIOA, BUTTON_B_PIN))
        current_raw_state |= BUTTON_B_MASK;
    if (!gpio_get(GPIOB, BUTTON_TURBO_A_PIN))
        current_raw_state |= BUTTON_TURBO_A_MASK;
    if (!gpio_get(GPIOB, BUTTON_TURBO_B_PIN))
        current_raw_state |= BUTTON_TURBO_B_MASK;

    for (int i = 0; i < 10; i++)
    {
        if (((current_raw_state >> i) & 1) == ((raw_state >> i) & 1))
        {
            if (counters[i] < DEBOUNCE_MS)
            {
                counters[i]++;
            }
            else
            {
                if (((current_raw_state >> i) & 1))
                {
                    debounced_state |= (1 << i);
                }
                else
                {
                    debounced_state &= ~(1 << i);
                }
            }
        }
        else
        {
            counters[i] = 0;
        }
    }
    raw_state = current_raw_state;

    // --- Turbo Logic ---
    static uint32_t last_turbo_toggle = 0;
    if (system_millis - last_turbo_toggle >= TURBO_PERIOD_MS)
    {
        turbo_toggle_state = !turbo_toggle_state;
        last_turbo_toggle = system_millis;
    }
}

// --- Main Application ---
int main(void)
{
    clock_setup();
    gpio_setup();
    systick_setup();

    usbd_dev = usbd_init(&stm32f103_usb_driver, &dev_descr, &config,
                         usb_strings, sizeof(usb_strings) / sizeof(char *),
                         usbd_control_buffer, sizeof(usbd_control_buffer));
    usbd_register_set_config_callback(usbd_dev, hid_set_config);

    uint16_t last_sent_state = 0xFFFF; // Force initial send

    while (1)
    {
        usbd_poll(usbd_dev);

        uint16_t current_state = debounced_state;
        uint16_t final_report_state = current_state;

        // Apply Turbo A logic
        if ((current_state & BUTTON_TURBO_A_MASK) && (current_state & BUTTON_A_MASK))
        {
            if (!turbo_toggle_state)
            {
                final_report_state &= ~BUTTON_A_MASK; // Force release
            }
        }

        // Apply Turbo B logic
        if ((current_state & BUTTON_TURBO_B_MASK) && (current_state & BUTTON_B_MASK))
        {
            if (!turbo_toggle_state)
            {
                final_report_state &= ~BUTTON_B_MASK; // Force release
            }
        }

        if (final_report_state != last_sent_state)
        {
            usbd_ep_write_packet(usbd_dev, 0x81, &final_report_state, 2);
            last_sent_state = final_report_state;
        }
    }

    return 0;
}