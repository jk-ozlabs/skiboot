
#define pr_fmt(fmt) "mctp-lpc: " fmt

#include <skiboot.h>
#include <lpc.h>
#include <mctp.h>
#include <device.h>
#include <interrupts.h>
#include <timer.h>
#include <timebase.h>

#include <libmctp/libmctp.h>

#define MCTP_LPC_KCS_IDR	0
#define MCTP_LPC_KCS_ODR	0
#define MCTP_LPC_KCS_STATUS	1

#define MCTP_LPC_KCS_STATUS_BMC_READY		(1<<7)
#define MCTP_LPC_KCS_STATUS_CHANNEL_READY	(1<<6)
#define MCTP_LPC_KCS_STATUS_IBF			(1<<1)
#define MCTP_LPC_KCS_STATUS_OBF			(1<<0)

#define MCTP_LPC_KCS_MSG_INIT		0x00
#define MCTP_LPC_KCS_MSG_TX_START	0x01
#define MCTP_LPC_KCS_MSG_RX_COMPLETE	0x02

struct mctp_binding_astlpc {
	struct mctp_binding	binding;
	struct mctp		*mctp;
	unsigned long		bus_id;
	struct lpc_client	lpc;
	unsigned int		version;

	int			kcs_base;
	int			kcs_stride;
	int			lpc_base;
	int			lpc_size;
	enum OpalLPCAddressType	lpc_type;
	uint32_t		irq;
	bool			irq_ok;

	bool			tx_pending;

	/* temporary buffers */
	unsigned char		rxbuf[MCTP_MTU];

	enum {
		MCTP_STATE_IDLE,
		MCTP_STATE_INIT,
		MCTP_STATE_RUNNING,
	}			state;

	/* queried from LPC mapping */
	uint32_t		rx_base;
	uint32_t		rx_size;
	uint32_t		tx_base;
	uint32_t		tx_size;

	struct timer		timer;

};

#define to_mctp_binding_astlpc(b) container_of(b, \
		struct mctp_binding_astlpc, binding)


#define MCTP_LPC_MAGIC		0x4d435450
#define MCTP_LPC_VERSION_MIN	1
#define MCTP_LPC_VERSION_CUR	1

struct mctp_lpcmap_hdr {
	uint32_t	magic;

	uint16_t	bmc_ver_min;
	uint16_t	bmc_ver_cur;
	uint16_t	host_ver_min;
	uint16_t	host_ver_cur;
	uint16_t	negotiated_ver;
	uint16_t	pad0;

	uint32_t	rx_offset;
	uint32_t	rx_size;
	uint32_t	tx_offset;
	uint32_t	tx_size;
} __attribute__((packed));

#ifndef offsetof
#define offsetof(type, member) \
	(unsigned long)(&((type *)(0)->member))
#endif

#define hdr_offset(member) (offsetof(struct mctp_lpcmap_hdr, member))

#ifndef min
#define min(x,y) ({ typeof(x) __x = (x); typeof(y) __y = (y); \
		__x < __y ? __x : __y; })
#endif
struct mctp_binding_astlpc mctp_lpc;

static uint32_t mctp_astlpc_map_read(uint32_t offset, int size, uint32_t *valp)
{
	uint32_t val, tmp;
	int rc, i;

	BUILD_ASSERT(size == 1 || size == 2 || size == 4);

	if (mctp_lpc.lpc_type == OPAL_LPC_FW) {
		rc = lpc_read(mctp_lpc.lpc_type, mctp_lpc.lpc_base + offset,
				&val, size);
	} else {
		val = 0;
		for (i = 0; i < size; i++) {
			rc = lpc_read(mctp_lpc.lpc_type,
					mctp_lpc.lpc_base + offset + i,
					&tmp, 1);
			if (rc)
				break;
			val <<= 8;
			val |= tmp;
		}
	}

	if (!rc)
		*valp = val;
	return rc;
}

static uint32_t mctp_astlpc_map_write(uint32_t offset, int size, uint32_t val)
{
	uint32_t tmp;
	int rc, i;

	BUILD_ASSERT(size == 1 || size == 2 || size == 4);

	if (mctp_lpc.lpc_type == OPAL_LPC_FW) {
		rc = lpc_write(mctp_lpc.lpc_type, mctp_lpc.lpc_base + offset,
				val, size);
	} else {
		for (i = 0; i < size; i++) {
			tmp = (val >> ((size - i - 1) * 8)) & 0xff;
			rc = lpc_write(mctp_lpc.lpc_type,
					mctp_lpc.lpc_base + offset + i,
					tmp, 1);
			if (rc)
				break;
		}
	}

	return rc;
}

static uint32_t mctp_astlpc_inl(uint32_t offset)
{
	uint32_t val;
	int rc;

	rc = mctp_astlpc_map_read(offset, 4, &val);
	return rc ? 0xffffffff : val;
}

static uint16_t mctp_astlpc_inw(uint32_t offset)
{
	uint32_t val;
	int rc;

	rc = mctp_astlpc_map_read(offset, 2, &val);
	return rc ? 0xffffffff : val;
}

static void mctp_astlpc_outw(uint32_t offset, uint16_t val)
{
	mctp_astlpc_map_write(offset, 2, val);
}

static uint8_t mctp_astlpc_kcs_read(int reg)
{
	return lpc_inb(mctp_lpc.kcs_base + (reg * mctp_lpc.kcs_stride));
}

static void mctp_astlpc_kcs_write(int reg, uint8_t val)
{
	lpc_outb(val, mctp_lpc.kcs_base + (reg * mctp_lpc.kcs_stride));
}

static void mctp_astlpc_kcs_send(uint8_t val)
{
	uint8_t status;

	/* todo: timeout */
	for (;;) {
		status = mctp_astlpc_kcs_read(MCTP_LPC_KCS_STATUS);
		if (!(status & MCTP_LPC_KCS_STATUS_IBF))
			break;
	}

	mctp_astlpc_kcs_write(MCTP_LPC_KCS_IDR, val);
}

static void mctp_astlpc_rx(void)
{
	struct mctp_pktbuf *pkt;
	uint32_t len, i;
	uint8_t *buf;
	int rc;

	rc = mctp_astlpc_map_read(mctp_lpc.rx_base, 4, &len);
	if (rc) {
		prerror("LPC RX read failed: %d\n", rc);
		return;
	}

	if (len > mctp_lpc.rx_size) {
		prerror("RX packet too large for mapping, dropping\n");
		return;
	}
	if (len > MCTP_MTU) {
		prerror("RX packet too large for buffer, dropping\n");
		return;
	}

	pkt = mctp_pktbuf_alloc(len);
	if (!pkt) {
		prerror("No packet buffers\n");
		return;
	}

	buf = pkt->data + pkt->mctp_hdr_off;
	rc = 0;

	/* Read packet data from the RX area - if possible, use 4-byte
	 * operations. */
	for (i = 0; i < len;) {
		uint32_t val;
		int sz;

		if (mctp_lpc.lpc_type == OPAL_LPC_FW) {
			sz = min(4, len - i);
			if (sz == 3)
				sz = 2;
		} else {
			sz = 1;
		}

		rc = lpc_read(mctp_lpc.lpc_type,
				mctp_lpc.lpc_base + mctp_lpc.rx_base + 4 + i,
				&val, sz);
		if (rc) {
			prerror("LPC RX failed: %d\n", rc);
			break;
		}

		switch (sz) {
		case 4:
			*(uint32_t *)(buf + i) = val;
			break;
		case 2:
			*(uint16_t *)(buf + i) = val & 0xffff;
			break;
		case 1:
			buf[i] = val & 0xff;
		}

		i += sz;
	}


	mctp_astlpc_kcs_send(MCTP_LPC_KCS_MSG_RX_COMPLETE);

	if (!rc)
		mctp_bus_rx(&mctp_lpc.binding, pkt);
}

static int mctp_astlpc_tx(struct mctp_binding *binding,
				struct mctp_pktbuf *pkt)
{
	struct mctp_binding_astlpc *mctp = to_mctp_binding_astlpc(binding);
	uint8_t *buf;
	uint32_t len;
	int i, rc;

	prerror("%s: %d\n", __func__, mctp_pktbuf_size(pkt));

	if (mctp_lpc.state != MCTP_STATE_RUNNING) {
		prerror("MCTP TX while state is %d\n", mctp_lpc.state);
		return -1;
	}

	len = mctp_pktbuf_size(pkt);
	buf = pkt->data + pkt->start;

	assert(len < 0xff);

	rc = mctp_astlpc_map_write(mctp_lpc.tx_base, 4, len);
	if (rc) {
		prerror("LPC TX write failed: %d\n", rc);
		return -1;
	}

	/* Write the actual packet to the TX area - if possible, use
	 * 4-byte operations. */
	for (i = 0; i < len;) {
		uint32_t val;
		int sz;

		if (mctp_lpc.lpc_type == OPAL_LPC_FW) {
			sz = min(4, len - i);
			if (sz == 3)
				sz = 2;
		} else {
			sz = 1;
		}

		switch (sz) {
		case 4:
			val = *(uint32_t *)(buf + i);
			break;
		case 2:
			val = *(uint16_t *)(buf + i);
			break;
		case 1:
			val = buf[i];
			break;
		default:
			assert(0);
		}

		rc = lpc_write(mctp_lpc.lpc_type,
				mctp_lpc.lpc_base + mctp_lpc.tx_base + 4 + i,
				val, sz);
		if (rc) {
			prerror("LPC TX write failed: %d\n", rc);
			break;
		}

		i += sz;
	}

	if (!rc) {
		mctp->tx_pending = true;
		mctp_binding_set_tx_enabled(binding, false);

		mctp_astlpc_kcs_send(MCTP_LPC_KCS_MSG_TX_START);
	}

	return !!rc;
}

static int mctp_astlpc_start_init(void)
{
	uint32_t reg;

	{
		prlog(PR_DEBUG, "BMC magic: %x, version %d:%d\n",
				mctp_astlpc_inw(hdr_offset(magic)),
				mctp_astlpc_inw(hdr_offset(bmc_ver_min)),
				mctp_astlpc_inw(hdr_offset(bmc_ver_cur)));
	}

	reg = be32_to_cpu(mctp_astlpc_inl(hdr_offset(magic)));
	if (reg != MCTP_LPC_MAGIC) {
		prerror("incorrect magic: 0x%08x\n", reg);
		return -1;
	}

	/* todo: check BMC version before starting init */

	/* populate our version info, and start negotiation by sending an
	 * init message
	 */
	mctp_astlpc_outw(hdr_offset(host_ver_min), MCTP_LPC_VERSION_MIN);
	mctp_astlpc_outw(hdr_offset(host_ver_cur), MCTP_LPC_VERSION_CUR);

	mctp_astlpc_kcs_send(MCTP_LPC_KCS_MSG_INIT);

	mctp_lpc.state = MCTP_STATE_INIT;

	return 0;
}

static int mctp_astlpc_start_channel(void)
{

	mctp_lpc.rx_base = be32_to_cpu(mctp_astlpc_inl(hdr_offset(rx_offset)));
	mctp_lpc.rx_size = be32_to_cpu(mctp_astlpc_inl(hdr_offset(rx_size)));
	mctp_lpc.tx_base = be32_to_cpu(mctp_astlpc_inl(hdr_offset(tx_offset)));
	mctp_lpc.tx_size = be32_to_cpu(mctp_astlpc_inl(hdr_offset(tx_size)));

	/* todo: sanity-check negotiated version */

	prerror("%s: rx @ 0x%x tx @ 0x%x\n", __func__,
			mctp_lpc.rx_base, mctp_lpc.tx_base);

	mctp_lpc.state = MCTP_STATE_RUNNING;
	mctp_binding_set_tx_enabled(&mctp_lpc.binding, true);

	return 0;
}

static void mctp_astlpc_poll_status(void)
{
	uint8_t status, data;

	/* Read KCS state and data registers. We've found that the only
	 * mechanism that clears a pending IRQ is a read from the data
	 * register, even if status[OBF] is not set. We don't use this value
	 * if OBF is clear.
	 */
	status = mctp_astlpc_kcs_read(MCTP_LPC_KCS_STATUS);
	data = mctp_astlpc_kcs_read(MCTP_LPC_KCS_IDR);

	if (!(status & MCTP_LPC_KCS_STATUS_BMC_READY)) {
		mctp_lpc.state = MCTP_STATE_IDLE;
		return;
	}

	switch (mctp_lpc.state) {

	case MCTP_STATE_IDLE:
		/* We know that BMC_READY is set (due to the check above),
		 * so start the init procedure */
		mctp_astlpc_start_init();
		return;

	case MCTP_STATE_INIT:
		if (!(status & MCTP_LPC_KCS_STATUS_CHANNEL_READY))
			return;

		mctp_astlpc_start_channel();
		break;

	case MCTP_STATE_RUNNING:
		break;

	default:
		prerror("invalid state %d?", mctp_lpc.state);
		mctp_lpc.state = MCTP_STATE_IDLE;
		return;
	}

	/* BMC has reset the channel? Start over. */
	if (!(status & MCTP_LPC_KCS_STATUS_CHANNEL_READY)) {
		mctp_lpc.state = MCTP_STATE_INIT;
		mctp_astlpc_start_init();
		return;
	}

	/* From here on, we know the channel is running, and our own stack is
	 * in running state too */

	/* no message pending? nothing to do */
	if (!(status & MCTP_LPC_KCS_STATUS_OBF))
		return;

	prerror("%s KCS message %x\n", __func__, data);

	switch (data) {
	case MCTP_LPC_KCS_MSG_TX_START:
		mctp_astlpc_rx();
		break;

	case MCTP_LPC_KCS_MSG_RX_COMPLETE:
		mctp_lpc.tx_pending = false;
		mctp_binding_set_tx_enabled(&mctp_lpc.binding, true);
		break;

	default:
		prerror("unknown message 0x%0x", data);

	}
}

static void mctp_astlpc_irq(uint32_t chip_id __unused,
		uint32_t irq_mask __unused)
{
	mctp_lpc.irq_ok = true;
	mctp_astlpc_poll_status();
}

static void mctp_timer_poll(struct timer *t __unused, void *data __unused,
		    uint64_t now __unused)
{
	mctp_astlpc_poll_status();

	if (!mctp_lpc.irq_ok)
		schedule_timer(&mctp_lpc.timer, msecs_to_tb(2000));
}

void mctp_astlpc_init(void)
{
	const struct dt_property *prop;
	struct dt_node *np;
	uint8_t eid = 9;
	uint32_t irq;

	np = dt_find_compatible_node(dt_root, NULL, "openbmc,mctp-lpc");
	if (!np)
		return;

	prop = dt_find_property(np, "reg");
	if (!prop) {
		prerror("No reg property?\n");
		return;
	}

	if (prop->len != 24) {
		prerror("reg property is invalid?");
		return;
	}

	if (dt_property_get_cell(prop, 0) != OPAL_LPC_IO) {
		prerror("Invalid LPC address type for KBC");
		return;
	}

	mctp_lpc.kcs_base = dt_property_get_cell(prop, 1);
	mctp_lpc.lpc_type = dt_property_get_cell(prop, 3);
	mctp_lpc.lpc_base = dt_property_get_cell(prop, 4);
	mctp_lpc.lpc_size = dt_property_get_cell(prop, 5);

	prop = dt_find_property(np, "kcs-stride");
	if (prop)
		mctp_lpc.kcs_stride = dt_property_get_cell(prop, 0);
	else
		mctp_lpc.kcs_stride = 1;

	irq = dt_prop_get_u32(np, "interrupts");
	mctp_lpc.lpc.interrupts = LPC_IRQ(irq);
	mctp_lpc.lpc.interrupt = mctp_astlpc_irq;

	mctp_lpc.binding.tx = mctp_astlpc_tx;

	mctp_lpc.state = MCTP_STATE_IDLE;

	lpc_register_client(dt_get_chip_id(np), &mctp_lpc.lpc,
			IRQ_ATTR_TARGET_OPAL);

	prlog(PR_ERR, "kcs 0x%x, lpc at 0x%x[%x], irq %d\n",
			mctp_lpc.kcs_base,
			mctp_lpc.lpc_base, mctp_lpc.lpc_size,
			irq);

	mctp_lpc.bus_id = mctp_register_bus(mctp, &mctp_lpc.binding, eid);

	/* schedule a timer to poll KCS events until we have interrupts up */
	init_timer(&mctp_lpc.timer, mctp_timer_poll, NULL);
	schedule_timer(&mctp_lpc.timer, msecs_to_tb(2000));

	mctp_astlpc_poll_status();

	mctp_test();
}
