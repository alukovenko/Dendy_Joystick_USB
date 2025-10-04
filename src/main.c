#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>

/* USB Device Descriptors */
#define USB_VID 0x1209
#define USB_PID 0x0001

/* HID Report Descriptor for Gamepad */
static const uint8_t hid_report_descriptor[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop Ctrls)
    0x09, 0x05,       // Usage (Game Pad)
    0xA1, 0x01,       // Collection (Application)
    0x09, 0x01,       //   Usage (Pointer)
    0xA1, 0x00,       //   Collection (Physical)
    0x09, 0x30,       //     Usage (X)
    0x09, 0x31,       //     Usage (Y)
    0x15, 0x00,       //     Logical Minimum (0)
    0x26, 0xFF, 0x00, //     Logical Maximum (255)
    0x35, 0x00,       //     Physical Minimum (0)
    0x46, 0xFF, 0x00, //     Physical Maximum (255)
    0x75, 0x08,       //     Report Size (8)
    0x95, 0x02,       //     Report Count (2)
    0x81, 0x02,       //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,             //   End Collection
    0x05, 0x09,       //   Usage Page (Button)
    0x19, 0x01,       //   Usage Minimum (0x01)
    0x29, 0x0A,       //   Usage Maximum (0x0A)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x01,       //   Logical Maximum (1)
    0x75, 0x01,       //   Report Size (1)
    0x95, 0x0A,       //   Report Count (10)
    0x81, 0x02,       //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x75, 0x06,       //   Report Size (6)
    0x95, 0x01,       //   Report Count (1)
    0x81, 0x03,       //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,             // End Collection
};

/* HID Report Structure */
struct hid_report
{
    uint8_t x;        // D-pad X axis (0=left, 127=center, 255=right)
    uint8_t y;        // D-pad Y axis (0=up, 127=center, 255=down)
    uint16_t buttons; // Button bits (bit 0 = button 1, etc.)
} __attribute__((packed));

/* USB Device Descriptor */
static const struct usb_device_descriptor dev_descr = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = 0,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

/* HID Function Descriptor with Report Descriptor */
static const uint8_t hid_function[] = {
    9,                                  /* bLength */
    0x21,                               /* bDescriptorType (HID) */
    0x00, 0x01,                         /* bcdHID (1.00) */
    0x00,                               /* bCountryCode */
    0x01,                               /* bNumDescriptors */
    0x22,                               /* bDescriptorType (Report) */
    sizeof(hid_report_descriptor), 0x00 /* wDescriptorLength */
};

/* HID Endpoint Descriptor */
static const struct usb_endpoint_descriptor hid_endpoint = {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x81,
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 4,
    .bInterval = 10,
};

/* HID Interface Descriptor */
static const struct usb_interface_descriptor hid_iface = {
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 1,
    .bInterfaceClass = USB_CLASS_HID,
    .bInterfaceSubClass = 1, /* boot */
    .bInterfaceProtocol = 0, /* gamepad */
    .iInterface = 0,
    .endpoint = &hid_endpoint,
    .extra = &hid_function,
    .extralen = sizeof(hid_function),
};

/* USB Interfaces */
static const struct usb_interface ifaces[] = {{
    .num_altsetting = 1,
    .altsetting = &hid_iface,
}};

/* USB Configuration Descriptor */
static const struct usb_config_descriptor config = {
    .bLength = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType = USB_DT_CONFIGURATION,
    .wTotalLength = 0,
    .bNumInterfaces = 1,
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = 0xC0,
    .bMaxPower = 0x32,
    .interface = ifaces,
};

/* USB String Descriptors */
static const char *usb_strings[] = {
    "Generic",
    "DendyJoystick",
    "DEMO",
};

/* Global USB device handle */
static usbd_device *usbd_dev;

/* USB Control Buffer */
static uint8_t usbd_control_buffer[128];

/* HID Control Request Handler */
static enum usbd_request_return_codes hid_control_request(usbd_device *usbd_dev,
                                                          struct usb_setup_data *req,
                                                          uint8_t **buf,
                                                          uint16_t *len,
                                                          void (**complete)(usbd_device *usbd_dev,
                                                                            struct usb_setup_data *req))
{
    (void)complete;
    (void)usbd_dev;

    if ((req->bmRequestType != 0x81) ||
        (req->bRequest != USB_REQ_GET_DESCRIPTOR) ||
        (req->wValue != 0x2200))
        return USBD_REQ_NOTSUPP;

    /* Handle HID report descriptor request */
    *buf = (uint8_t *)hid_report_descriptor;
    *len = sizeof(hid_report_descriptor);

    return USBD_REQ_HANDLED;
}

/* USB Set Configuration Handler */
static void hid_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
    (void)wValue;

    usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_INTERRUPT, 4, NULL);

    usbd_register_control_callback(
        usbd_dev,
        USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
        USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
        hid_control_request);
}

/* Initialize USB */
static void usb_setup(void)
{
    rcc_periph_clock_enable(RCC_USB);
    rcc_periph_clock_enable(RCC_GPIOA);

    /* Setup USB pins on PA11 and PA12 */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO11 | GPIO12);

    usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev_descr, &config,
                         usb_strings, 3, usbd_control_buffer,
                         sizeof(usbd_control_buffer));

    usbd_register_set_config_callback(usbd_dev, hid_set_config);
}

/* Send HID report */
static void send_hid_report(struct hid_report *report)
{
    usbd_ep_write_packet(usbd_dev, 0x81, report, sizeof(struct hid_report));
}

/*
Bluepill board based on STM32F103C8T6 microcontroller. The project implements
a USB HID device using libopencm3 that simulates a gamepad with the following buttons:

DPad Up	          PA0	            BUTTON_UP_PIN
DPad Down	        PA1	            BUTTON_DOWN_PIN
DPad Left	        PA2	            BUTTON_LEFT_PIN
DPad Right	      PA3	            BUTTON_RIGHT_PIN
Select	          PA4	            BUTTON_SELECT_PIN
Start	            PA5	            BUTTON_START_PIN
Button A	        PA6	            BUTTON_A_PIN
Button B	        PA7	            BUTTON_B_PIN
Turbo A	          PB0	            BUTTON_TURBO_A_PIN
Turbo B	          PB1	            BUTTON_TURBO_B_PIN

COMMON GROUND G GND Connect to any GND pin on BluePill

All buttons should connect to GND when pressed (active low configuration), and require internal pull-up resistors to function correctly.

Turbo A and Turbo B buttons are "rapid fire" buttons. When held down, they
simulate repeated presses of Button A and Button B respectively at 15 Hz.
*/

/* SysTick interrupt handler */
void sys_tick_handler(void)
{
    gpio_toggle(GPIOC, GPIO13);
}

static void setup_systick(void)
{
    /* Setup SysTick to fire every 500ms */
    /* Assuming 72MHz system clock (default for STM32F103) */
    systick_set_reload(72000000 / 2 - 1); /* 500ms = 0.5Hz */
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_counter_enable();
    systick_interrupt_enable();
}

int main(void)
{
    /* Setup system clock to 72MHz */
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

    /* Enable GPIOC clock for LED */
    rcc_periph_clock_enable(RCC_GPIOC);

    /* Set GPIO13 (PC13, LED pin) to 'output push-pull' */
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

    /* Setup SysTick timer for LED blinking */
    setup_systick();

    /* Initialize USB */
    usb_setup();

    /* Create a test HID report */
    struct hid_report report = {
        .x = 127,    /* Center X */
        .y = 127,    /* Center Y */
        .buttons = 0 /* No buttons pressed */
    };

    /* Main loop */
    while (1)
    {
        /* Handle USB events */
        usbd_poll(usbd_dev);

        /* Send a test report every ~10ms */
        static uint32_t last_report = 0;
        if (systick_get_countflag())
        {
            last_report++;
            if (last_report >= 20)
            { /* Every ~10ms (500ms/50) */
                last_report = 0;
                send_hid_report(&report);
            }
        }
    }

    return 0;
}
