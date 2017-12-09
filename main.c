/*
 * Copyright (c) 2017, Andrew Chen <andrew.chuanye.chen@gmail.com>
 *
 * This file is based on part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>

static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = USB_CLASS_CDC,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0483,
	.idProduct = 0x5740,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

/*
 * This notification endpoint isn't implemented. According to CDC spec its
 * optional, but its absence causes a NULL pointer dereference in Linux
 * cdc_acm driver.
 */
static const struct usb_endpoint_descriptor comm_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x81,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 16,
	.bInterval = 255,
}};

static const struct usb_endpoint_descriptor data_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x02,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x82,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}};


static const struct {
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
	.header = {
		.bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
		.bcdCDC = 0x0110,
	},
	.call_mgmt = {
		.bFunctionLength =
			sizeof(struct usb_cdc_call_management_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
		.bmCapabilities = 0,
		.bDataInterface = 1,
	},
	.acm = {
		.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_ACM,
		.bmCapabilities = 0,
	},
	.cdc_union = {
		.bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_UNION,
		.bControlInterface = 0,
		.bSubordinateInterface0 = 1,
	 },
};

static const struct usb_interface_descriptor comm_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_CDC,
	.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
	.iInterface = 0,

	.endpoint = comm_endp,

	.extra = &cdcacm_functional_descriptors,
	.extralen = sizeof(cdcacm_functional_descriptors),
}};


static const struct usb_interface_descriptor data_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_DATA,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = data_endp,
}};

static const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = comm_iface,
}, {
	.num_altsetting = 1,
	.altsetting = data_iface,
}};


static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 2,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char *usb_strings[] = {
	"",
	"SPI to USB",
	"",
};

/* Buffer to be used for control requests. */
static uint8_t usbd_control_buffer[128];

static int cdcacm_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
		uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req)) {
	(void)usbd_dev;
	(void)req;
	(void)buf;
	(void)len;
	(void)complete;

	return 1;
}

static bool usb_txne;

static void cdcacm_data_tx_cb(usbd_device *usbd_dev, uint8_t ep) {
	(void)ep;
	(void)usbd_dev;

	usb_txne = false;
}

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue) {
	(void)wValue;

	usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL); // comm_endp
	usbd_ep_setup(usbd_dev, 0x02, USB_ENDPOINT_ATTR_BULK, 64, NULL); // data_endp
	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_tx_cb); // data_endp

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				cdcacm_control_request);
}

// circular buffer for receiving data from spi
static uint8_t recv_buf[1024];
static uint8_t head, tail;

inline int min(int a, int b) {
	return (a < b) ? a : b;
}

static void usb_transmit_task(usbd_device *usbd_dev) {
	if (head == tail) return;
	if (usb_txne) return;
	usb_txne = true;

	uint8_t _tail = tail;
	if (_tail > head) {
		int len = min(64, _tail - head);
		usbd_ep_write_packet(usbd_dev, 0x82, &recv_buf[head], len);
		head += len;
	} else {
		int len = min(64, sizeof(recv_buf) - head);
		usbd_ep_write_packet(usbd_dev, 0x82, &recv_buf[head], len);
		head = 0;
	}
}

void spi1_isr(void) {
	if (SPI_SR(SPI1) & SPI_SR_RXNE) {
		recv_buf[tail] = SPI_DR(SPI1);
		tail = (tail + 1) % sizeof(recv_buf);
	}
}

static void spi_setup(void);

// reset SPI event handler
void exti3_isr(void) {
	EXTI_PR = EXTI3; // clear pending flag

	RCC_APB2RSTR = RCC_APB2RSTR_SPI1RST;
	RCC_APB2RSTR = 0;

	spi_setup();
}

static void clock_setup(void) {
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_SPI1);
}

static void exti_setup(void) {
	// A3: reset
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
	              GPIO_CNF_INPUT_PULL_UPDOWN, GPIO3);
	gpio_clear(GPIOA, GPIO3); // pull down

	nvic_enable_irq(NVIC_EXTI3_IRQ);

	exti_set_trigger(EXTI3, EXTI_TRIGGER_RISING);
	exti_select_source(EXTI3, GPIOA);
	exti_enable_request(EXTI3);
}

static void spi_setup(void) {
	// A5: SCLK
	// A7: MOSI
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
	              GPIO_CNF_INPUT_PULL_UPDOWN, GPIO5 | GPIO7);
	gpio_clear(GPIOA, GPIO5 | GPIO7); // pull down

	nvic_enable_irq(NVIC_SPI1_IRQ);

	spi_set_slave_mode(SPI1);
	spi_set_standard_mode(SPI1, 0);
	spi_set_receive_only_mode(SPI1);
	spi_set_dff_8bit(SPI1);
	spi_send_lsb_first(SPI1);
	spi_enable_software_slave_management(SPI1);
	spi_set_nss_low(SPI1);
	spi_enable_rx_buffer_not_empty_interrupt(SPI1);
	spi_enable(SPI1);
}

int main(void) {
	clock_setup();
	exti_setup();
	spi_setup();

	usbd_device *usbd_dev;
	usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config,
			usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));

	usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);

	while (1) {
		usbd_poll(usbd_dev);

		usb_transmit_task(usbd_dev);
	}
}
