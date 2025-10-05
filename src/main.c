#include <stdint.h>
#include <stddef.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>

/* Function Prototypes */
void sys_tick_handler(void);
static void setup_systick(void);

/* System Constants */
#define SYSTEM_CLOCK_HZ 72000000
#define SYSTICK_FREQUENCY_HZ 1000
#define LED_TOGGLE_INTERVAL_MS 1000
#define USB_CONTROL_BUFFER_SIZE 128

/* USB Device Descriptors - using pid.codes VID/PID */
#define USB_VID 0x1209
#define USB_PID 0x0001

/* Button Pin Definitions */
#define BUTTON_UP_PIN GPIO0      // PA0 - DPad Up
#define BUTTON_DOWN_PIN GPIO1    // PA1 - DPad Down
#define BUTTON_LEFT_PIN GPIO2    // PA2 - DPad Left
#define BUTTON_RIGHT_PIN GPIO3   // PA3 - DPad Right
#define BUTTON_SELECT_PIN GPIO4  // PA4 - Select
#define BUTTON_START_PIN GPIO5   // PA5 - Start
#define BUTTON_A_PIN GPIO6       // PA6 - Button A
#define BUTTON_B_PIN GPIO7       // PA7 - Button B
#define BUTTON_TURBO_A_PIN GPIO0 // PB0 - Turbo A
#define BUTTON_TURBO_B_PIN GPIO1 // PB1 - Turbo B

/* Button bit positions in HID report - NES Controller Layout */
#define BUTTON_A_BIT 0      // Button A
#define BUTTON_B_BIT 1      // Button B
#define BUTTON_SELECT_BIT 2 // Select
#define BUTTON_START_BIT 3  // Start
#define BUTTON_UP_BIT 4     // D-pad Up
#define BUTTON_DOWN_BIT 5   // D-pad Down
#define BUTTON_LEFT_BIT 6   // D-pad Left
#define BUTTON_RIGHT_BIT 7  // D-pad Right

/* HID Report Descriptor for Gamepad with Digital Buttons Only */
static const uint8_t hid_report_descriptor[] = {
    0x05, 0x01, // Usage Page (Generic Desktop)
    0x09, 0x05, // Usage (Game Pad)
    0xA1, 0x01, // Collection (Application)

    // Button Report (8 buttons)
    0x05, 0x09, //   Usage Page (Button)
    0x19, 0x01, //   Usage Minimum (Button 1)
    0x29, 0x08, //   Usage Maximum (Button 8)
    0x15, 0x00, //   Logical Minimum (0)
    0x25, 0x01, //   Logical Maximum (1)
    0x75, 0x01, //   Report Size (1 bit)
    0x95, 0x08, //   Report Count (8 buttons)
    0x81, 0x02, //   Input (Data,Var,Abs)

    // Padding for button byte alignment
    0x75, 0x08, //   Report Size (8 bits)
    0x95, 0x01, //   Report Count (1)
    0x81, 0x03, //   Input (Const,Var,Abs) - padding

    0xC0, // End Collection
};

/* HID Report Structure - Buttons Only */
struct hid_report
{
    uint16_t buttons; // Button bits (8 buttons + 8 bits padding)
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
    .wMaxPacketSize = 2, /* 2 bytes for button report */
    .bInterval = 1,      /* 1ms interval for maximum responsiveness */
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
    "DN001",
};

/* Global USB device handle */
static usbd_device *usbd_dev;

/* USB Control Buffer */
static uint8_t usbd_control_buffer[USB_CONTROL_BUFFER_SIZE];

/* Turbo timing variables */
static uint32_t turbo_counters[2] = {0, 0}; // [0] = Turbo A, [1] = Turbo B
static uint8_t turbo_states[2] = {0, 0};    // Current turbo output states
static uint8_t turbo_prev[2] = {0, 0};      // Previous button states for edge detection

/* Debouncing variables */
#define DEBOUNCE_TIME_TICKS 2 // 2 ticks * 10ms = 20ms debounce for cheap switches
#define NUM_BUTTONS 10        // 8 regular buttons (PA0-PA7) + 2 turbo buttons (PB0-PB1)
#define NUM_REGULAR_BUTTONS 8 // PA0-PA7
#define NUM_TURBO_BUTTONS 2   // PB0-PB1

/* Button state structure */
struct button_state
{
    uint32_t port;            // GPIO port
    uint16_t pin;             // GPIO pin
    uint8_t bit;              // HID button bit (0xFF for turbo buttons)
    uint8_t debounce_counter; // Debounce counter
    uint8_t state;            // Stable debounced state
    uint8_t raw_prev;         // Previous raw reading
};

/* Global button configuration and state */
static struct button_state buttons[NUM_BUTTONS] = {
    /* Regular buttons (0-7) */
    {GPIOA, BUTTON_UP_PIN, BUTTON_UP_BIT, 0, 0, 0},
    {GPIOA, BUTTON_DOWN_PIN, BUTTON_DOWN_BIT, 0, 0, 0},
    {GPIOA, BUTTON_LEFT_PIN, BUTTON_LEFT_BIT, 0, 0, 0},
    {GPIOA, BUTTON_RIGHT_PIN, BUTTON_RIGHT_BIT, 0, 0, 0},
    {GPIOA, BUTTON_SELECT_PIN, BUTTON_SELECT_BIT, 0, 0, 0},
    {GPIOA, BUTTON_START_PIN, BUTTON_START_BIT, 0, 0, 0},
    {GPIOA, BUTTON_A_PIN, BUTTON_A_BIT, 0, 0, 0},
    {GPIOA, BUTTON_B_PIN, BUTTON_B_BIT, 0, 0, 0},
    /* Turbo buttons (8-9) - don't map to HID bits, just for debouncing */
    {GPIOB, BUTTON_TURBO_A_PIN, 0xFF, 0, 0, 0}, // 0xFF = no HID bit
    {GPIOB, BUTTON_TURBO_B_PIN, 0xFF, 0, 0, 0}  // 0xFF = no HID bit
};

/* Setup GPIO pins for buttons */
static void setup_gpio(void)
{
    /* Enable clocks for GPIOA and GPIOB */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);

    /* Configure PA0-PA7 as input with pull-up */
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_PULL_UPDOWN,
                  BUTTON_UP_PIN | BUTTON_DOWN_PIN | BUTTON_LEFT_PIN | BUTTON_RIGHT_PIN |
                      BUTTON_SELECT_PIN | BUTTON_START_PIN | BUTTON_A_PIN | BUTTON_B_PIN);

    /* Enable pull-up resistors for PA0-PA7 */
    gpio_set(GPIOA, BUTTON_UP_PIN | BUTTON_DOWN_PIN | BUTTON_LEFT_PIN | BUTTON_RIGHT_PIN |
                        BUTTON_SELECT_PIN | BUTTON_START_PIN | BUTTON_A_PIN | BUTTON_B_PIN);

    /* Configure PB0-PB1 as input with pull-up */
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_PULL_UPDOWN,
                  BUTTON_TURBO_A_PIN | BUTTON_TURBO_B_PIN);

    /* Enable pull-up resistors for PB0-PB1 */
    gpio_set(GPIOB, BUTTON_TURBO_A_PIN | BUTTON_TURBO_B_PIN);
}

/* Read button states and update HID report */
static void update_gamepad_state(struct hid_report *report)
{
    uint16_t hid_buttons = 0;

    /* Process debounced button readings */
    for (int i = 0; i < NUM_BUTTONS; i++)
    {
        /* Read raw button state (active low) */
        uint8_t button_raw = !gpio_get(buttons[i].port, buttons[i].pin);

        /* Debounce logic */
        if (button_raw != buttons[i].raw_prev)
        {
            /* Button state changed, start/reset debounce counter */
            buttons[i].debounce_counter = 0;
        }
        else
        {
            if (buttons[i].debounce_counter < DEBOUNCE_TIME_TICKS)
            {
                /* Button state stable, increment counter */
                buttons[i].debounce_counter++;
            }
            else
            {
                /* If debounce time reached, update stable state */
                buttons[i].state = button_raw;
            }
        }

        /* Apply debounced button state to HID report (only for regular buttons) */
        if (i < NUM_REGULAR_BUTTONS && buttons[i].state && buttons[i].bit != 0xFF)
        {
            hid_buttons |= (1 << buttons[i].bit);
        }

        /* Store previous raw reading */
        buttons[i].raw_prev = button_raw;
    }

    /* Handle turbo buttons with 15Hz rate - using debounced states */
    uint8_t turbo_pressed[2] = {
        buttons[8].state, // Turbo A (debounced)
        buttons[9].state  // Turbo B (debounced)
    };

    uint8_t turbo_button_bits[2] = {BUTTON_A_BIT, BUTTON_B_BIT};

    /* Process each turbo button independently */
    for (int i = 0; i < 2; i++)
    {
        /* Check for button state change first */
        uint8_t button_changed = (turbo_pressed[i] != turbo_prev[i]);

        if (turbo_pressed[i])
        {
            /* Button is currently pressed */
            if (button_changed)
            {
                /* First press detection - fire immediately and reset timing */
                turbo_counters[i] = 0;
                turbo_states[i] = 1; /* Start with button pressed */
            }
            else
            {
                /* Continue turbo timing - increment counter each iteration */
                turbo_counters[i]++;
                if (turbo_counters[i] >= 3)
                { /* ~3 ticks at 10ms intervals = 30ms for ~16Hz turbo */
                    turbo_counters[i] = 0;
                    turbo_states[i] = !turbo_states[i];
                }
            }

            /* Apply turbo state to HID buttons */
            if (turbo_states[i])
            {
                hid_buttons |= (1 << turbo_button_bits[i]);
            }
        }
        else
        {
            /* Button is released - always reset turbo state completely */
            turbo_counters[i] = 0;
            turbo_states[i] = 0;
        }

        /* Update previous button state for edge detection */
        turbo_prev[i] = turbo_pressed[i];
    }

    /* Update report */
    report->buttons = hid_buttons;
}

/* HID Control Request Handler */
static enum usbd_request_return_codes hid_control_request(usbd_device *dev,
                                                          struct usb_setup_data *req,
                                                          uint8_t **buf,
                                                          uint16_t *len,
                                                          void (**complete)(usbd_device *usbd_dev,
                                                                            struct usb_setup_data *req))
{
    (void)complete;
    (void)dev;

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
static void hid_set_config(usbd_device *dev, uint16_t wValue)
{
    (void)wValue;

    usbd_ep_setup(dev, 0x81, USB_ENDPOINT_ATTR_INTERRUPT, 2, NULL); /* 2-byte reports */

    usbd_register_control_callback(
        dev,
        USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
        USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
        hid_control_request);

    /* Setup SysTick after USB configuration is complete */
    setup_systick();
}

/* Initialize USB */
static void usb_setup(void)
{
    rcc_periph_clock_enable(RCC_USB);
    rcc_periph_clock_enable(RCC_GPIOA);

    /*
     * This is a somewhat common cheap hack to trigger device re-enumeration
     * on startup. Assuming a fixed external pullup on D+, (For USB-FS)
     * setting the pin to output, and driving it explicitly low effectively
     * "removes" the pullup. The subsequent USB init will "take over" the
     * pin, and it will appear as a proper pullup to the host.
     * The magic delay is somewhat arbitrary, no guarantees on USBIF
     * compliance here, but "it works" in most places.
     */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
    gpio_clear(GPIOA, GPIO12);
    for (unsigned i = 0; i < 800000; i++)
    {
        __asm__("nop");
    }

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

/* Timing variables */
static volatile uint32_t system_millis = 0;
static volatile uint8_t led_toggle_flag = 0;

/* SysTick interrupt handler */
void sys_tick_handler(void)
{
    static uint32_t tick_counter = 0;

    tick_counter++;

    /* Update millisecond counter (approximate, since we're running at ~100Hz) */
    system_millis += 10;

    /* Toggle LED every ~1 second (50 ticks * 10ms = 500ms, giving 1Hz blink rate) */
    if (tick_counter % 50 == 0)
    {
        led_toggle_flag = 1;
    }
}

static void setup_systick(void)
{
    /* Setup SysTick similar to stable implementation */
    /* Using AHB/8 clock source for more reliable operation */
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
    /* At 72MHz, AHB/8 = 9MHz, so 99999 gives ~100Hz (10ms intervals) */
    systick_set_reload(99999);
    systick_counter_enable();
    systick_interrupt_enable();
}

int main(void)
{
    /* Setup 72MHz system clock using 8MHz external crystal */
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

    /* Enable GPIOC clock for LED */
    rcc_periph_clock_enable(RCC_GPIOC);

    /* Set GPIO13 (PC13, LED pin) to 'output push-pull' */
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

    /* Initialize gamepad GPIO */
    setup_gpio();

    /* Initialize USB HID device */
    usb_setup();

    /* Initialize HID report */
    struct hid_report report = {
        .buttons = 0 /* No buttons pressed */
    };

    struct hid_report last_report = {
        .buttons = 0 /* Previous state */
    };

    uint32_t last_report_time = 0;

    /* Main loop with full gamepad functionality */
    while (1)
    {
        /* Handle USB events */
        usbd_poll(usbd_dev);

        /* Handle LED blinking - only if SysTick is running */
        if (led_toggle_flag)
        {
            led_toggle_flag = 0;
            gpio_toggle(GPIOC, GPIO13);
        }

        /* Read current gamepad state every loop iteration */
        update_gamepad_state(&report);

        /* Send HID report immediately on state change OR every 50ms for turbo/keepalive */
        /* Only do this if SysTick is running (system_millis will be non-zero after first tick) */
        if (system_millis > 0)
        {
            uint8_t state_changed = (report.buttons != last_report.buttons);
            uint8_t periodic_update = (system_millis - last_report_time >= 50);

            if (state_changed || periodic_update)
            {
                last_report_time = system_millis;
                last_report = report;

                /* Send the report */
                send_hid_report(&report);
            }
        }
    }

    return 0;
}
