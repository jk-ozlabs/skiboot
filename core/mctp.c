
#include <skiboot.h>
#include <mctp.h>
#include <libmctp/libmctp.h>

void mctp_prlog(int level, const char *fmt, ...);
int vprlog(int log_level, const char *fmt, va_list ap);

void mctp_prlog(int level, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vprlog(level, fmt, ap);
	va_end(ap);
}

static void *mctp_realloc(void *ptr, size_t size)
{
	return realloc(ptr, size);
}

void mctp_test(void)
{
	uint8_t buf[] = "\x01meep";
	mctp_message_tx(mctp, 8, buf, strlen(buf));
}

static void mctp_rx(uint8_t src_eid, void *data, void *msg, size_t len)
{
	(void)(data);
	prlog(PR_DEBUG, "MCTP RX: eid 0x%02x, len %ld, byte 0x%02x\n",
			src_eid, len, len > 1 ? *(uint8_t *)msg : 0xff);
}

void opal_mctp_init(void)
{
	mctp_set_alloc_ops(NULL, NULL, mctp_realloc);
	mctp = mctp_init();
	mctp_set_rx_all(mctp, mctp_rx, NULL);
}

