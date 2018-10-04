#include "general.h"
#include "gdb_if.h"
#include "serialno.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/usb/dfu.h>
#include <stdlib.h>

usbd_device * usbdev;

static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0xFE,		/* Application Specific Class */
	.bDeviceSubClass = 01,		/* Device Firmware Update Subclass */
	.bDeviceProtocol = 0,		/* Interface Association */
	.bMaxPacketSize0 = 64,
	.idVendor = 0x1209,
	.idProduct = 0x5555,
	.bcdDevice = 0x0100,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

const struct usb_dfu_descriptor dfu_function = {
	.bLength = sizeof(struct usb_dfu_descriptor),
	.bDescriptorType = DFU_FUNCTIONAL,
	.bmAttributes = USB_DFU_CAN_DOWNLOAD | USB_DFU_WILL_DETACH,
	.wDetachTimeout = 255,
	.wTransferSize = 1024,
	.bcdDFUVersion = 0x011A,
};

const struct usb_interface_descriptor dfu_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = 0xFE,
	.bInterfaceSubClass = 1,
	.bInterfaceProtocol = 1,
	.iInterface = 4,

	.extra = &dfu_function,
	.extralen = sizeof(dfu_function),
};

static const struct usb_interface ifaces[] = {
    {
        .num_altsetting = 1,
        .altsetting = &dfu_iface,
    }
};

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

#if defined(STM32L0) || defined(STM32F3) || defined(STM32F4)
char serial_no[13];
#else
char serial_no[9];
#endif

static const char *usb_strings[] = {
	"Black Sphere Technologies",
	BOARD_IDENT,
	serial_no,
	DFU_IDENT,
};

static void dfu_detach_complete(usbd_device *dev, struct usb_setup_data *req)
{
	(void)dev;
	(void)req;

	platform_request_boot();

	/* Reset core to enter bootloader */
	scb_reset_core();
}

static int dfu_control_request(usbd_device *dev,
		struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
		void (**complete)(usbd_device *dev, struct usb_setup_data *req))
{
	(void)dev;
	(void)complete;
	(void)buf;
	(void)len;

	switch(req->bRequest) {
	case DFU_GETSTATUS:
		if(req->wIndex == 0) {
			(*buf)[0] = DFU_STATUS_OK;
			(*buf)[1] = 0;
			(*buf)[2] = 0;
			(*buf)[3] = 0;
			(*buf)[4] = STATE_APP_IDLE;
			(*buf)[5] = 0;	/* iString not used here */
			*len = 6;

			return 1;
		}
		return 0;
	case DFU_DETACH:
		if(req->wIndex == 0) {
			*complete = dfu_detach_complete;
			return 1;
		}
		return 0;
	}
	return 0;
}

static void dfu_set_config(usbd_device *dev, uint16_t wValue)
{
    (void)wValue;
    usbd_register_control_callback(dev,
			USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
			USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
			dfu_control_request);
}

/* We need a special large control buffer for this device: */
uint8_t usbd_control_buffer[256];

void dfu_init(void)
{
	serialno_read(serial_no);

	usbdev = usbd_init(&USB_DRIVER, &dev, &config, usb_strings,
			    sizeof(usb_strings)/sizeof(char *),
			    usbd_control_buffer, sizeof(usbd_control_buffer));

	usbd_register_set_config_callback(usbdev, dfu_set_config);

	nvic_set_priority(USB_IRQ, IRQ_PRI_USB);
	nvic_enable_irq(USB_IRQ);
}

void USB_ISR(void)
{
	usbd_poll(usbdev);
}
