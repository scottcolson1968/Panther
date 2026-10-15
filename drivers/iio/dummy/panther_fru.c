// SPDX-License-Identifier: GPL-2.0
/*
 * Panther FMC FRU IIO platform driver
 *
 * Exposes the VITA-57.1 / IPMI "Platform Management FRU Information" Board
 * Info Area held in the FMC FRU EEPROM (at24 24c02 on i2c-1 @ 0x50) as
 * read/write IIO *device* attributes:
 *
 *   manufacturer     -> Board Area Manufacturer
 *   product_name     -> Board Area Product Name   (the "Device Name")
 *   serial_number    -> Board Area Serial Number
 *   board_revision   -> Board Area Part Number
 *
 * The EEPROM is reached through the at24 nvmem device (no second I2C client),
 * referenced from the device tree via an nvmem phandle:
 *
 *   fmc_fru {
 *       compatible = "sd,fru";
 *       nvmem = <&fmc_eeprom>;      // phandle to the eeprom@50 node
 *       nvmem-names = "fru";
 *   };
 *
 * On write the Board Info Area is rebuilt in place (type/length bytes,
 * zero checksums, area offsets) while any Internal/Chassis/Product areas are
 * rejected and any MultiRecord Area is preserved verbatim, so the resulting
 * EEPROM image stays byte-compatible with ADI's fru-dump tool.  The FRU
 * encode/decode logic mirrors the BSD-licensed fru_tools (fru.c) so the two
 * agree on the wire format.
 *
 * Author: Scott Colson <colsons@sd-star.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/nvmem-consumer.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
/* network identity attributes (ip_address / mac_address) */
#include <linux/kmod.h>
#include <linux/inet.h>
#include <linux/netdevice.h>
#include <linux/inetdevice.h>
#include <linux/etherdevice.h>

#define FRU_EEPROM_SIZE		256
#define FRU_PAGE_SIZE		8	/* 24c02 write page */
#define FRU_STR_MAX		63	/* type/length field only carries 6 bits */

/* Type codes, FRU spec section 13 "TYPE/LENGTH BYTE FORMAT" */
#define FRU_TYPE_BINARY		0
#define FRU_TYPE_BCD		1
#define FRU_TYPE_SIXBIT		2
#define FRU_TYPE_ASCII		3

#define FRU_TL_TYPE(b)		(((b) >> 6) & 0x3)
#define FRU_TL_LEN(b)		((b) & 0x3F)
#define FRU_ENDMARK		0xC1	/* type/length byte: no more fields */

/*
 * Board Info Area standard string fields, in on-EEPROM order.  All five are
 * read and re-emitted so field positions are preserved on write; only the
 * first four are exposed as IIO attributes.
 */
enum fru_field {
	FRU_MANUFACTURER = 0,
	FRU_PRODUCT_NAME,
	FRU_SERIAL_NUMBER,
	FRU_PART_NUMBER,	/* exposed as "board_revision" */
	FRU_FILE_ID,		/* preserved verbatim, no attribute */
	FRU_NUM_STD_FIELDS,
};

struct panther_fru {
	struct nvmem_device	*nvmem;
	struct mutex		lock;
};

/* -------------------------------------------------------------------------
 * FRU checksum: the "zero checksum" of the FRU spec is chosen so the modulo
 * 256 sum of a region (including the checksum byte) is zero.
 * ------------------------------------------------------------------------- */
static u8 fru_sum(const u8 *d, size_t n)
{
	u8 s = 0;

	while (n--)
		s += *d++;
	return s;
}

/* -------------------------------------------------------------------------
 * 6-bit ASCII unpacking (FRU spec section 13) - only needed on read, in case
 * a field was written by another tool in packed form.  Writes always emit
 * plain 8-bit ASCII.
 * ------------------------------------------------------------------------- */
static int fru_six2ascii(const u8 *buf, int size, char *out, int outsz)
{
	int i, n = 0;

	for (i = 0; i < size && n < outsz - 1; i += 3) {
		out[n++] = (buf[i] & 0x3F) + 0x20;
		if (i + 1 < size && n < outsz - 1)
			out[n++] = (((buf[i] & 0xC0) >> 6) |
				    ((buf[i + 1] & 0x0F) << 2)) + 0x20;
		if (i + 2 < size && n < outsz - 1)
			out[n++] = (((buf[i + 1] & 0xF0) >> 4) |
				    ((buf[i + 2] & 0x03) << 4)) + 0x20;
		if (i + 2 < size && n < outsz - 1)
			out[n++] = ((buf[i + 2] & 0xFC) >> 2) + 0x20;
	}
	while (n > 0 && (out[n - 1] == ' ' || out[n - 1] == '\0'))
		n--;
	out[n] = '\0';
	return n;
}

/*
 * Decode one type/length field at *p into a printable string.
 * Returns the number of EEPROM bytes consumed (TL byte + data), or a
 * negative errno.  end bounds the buffer so we never walk off the EEPROM.
 */
static int fru_decode(const u8 *p, const u8 *end, char *out, int outsz)
{
	u8 tl, type, len;

	if (p >= end)
		return -EINVAL;
	tl = p[0];
	if (tl == FRU_ENDMARK) {
		out[0] = '\0';
		return 0;
	}
	type = FRU_TL_TYPE(tl);
	len = FRU_TL_LEN(tl);
	if (p + 1 + len > end)
		return -EINVAL;

	switch (type) {
	case FRU_TYPE_ASCII:
	case FRU_TYPE_BINARY:	/* treat unspecified/binary as raw bytes */
		len = min_t(int, len, outsz - 1);
		memcpy(out, p + 1, len);
		out[len] = '\0';
		break;
	case FRU_TYPE_SIXBIT:
		fru_six2ascii(p + 1, len, out, outsz);
		break;
	case FRU_TYPE_BCD:
	default:
		return -EOPNOTSUPP;
	}
	return len + 1;
}

/* Locate the Board Info Area in a FRU image, validating the common header. */
static int fru_board_area(const u8 *buf, const u8 **board, const u8 **end)
{
	u8 off, blen;

	if (buf[0] != 0x01)		/* common header format version */
		return -ENODATA;	/* blank / uninitialised EEPROM */
	if (fru_sum(buf, 8) != 0)	/* common header checksum */
		return -EBADMSG;
	off = buf[3];			/* board area offset, 8-byte units */
	if (!off)
		return -ENOENT;
	if ((size_t)off * 8 + 2 > FRU_EEPROM_SIZE)
		return -EINVAL;
	*board = buf + off * 8;
	if ((*board)[0] != 0x01)	/* board area format version */
		return -EBADMSG;
	blen = (*board)[1];		/* board area length, 8-byte units */
	if ((size_t)(off + blen) * 8 > FRU_EEPROM_SIZE)
		return -EINVAL;
	*end = *board + blen * 8;
	return 0;
}

/* Read one standard Board Info string field into out (NUL-terminated). */
static int fru_read_field(struct panther_fru *st, enum fru_field field,
			  char *out, int outsz)
{
	u8 *buf;
	const u8 *board, *end, *p;
	int i, ret;

	buf = kmalloc(FRU_EEPROM_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = nvmem_device_read(st->nvmem, 0, FRU_EEPROM_SIZE, buf);
	if (ret < 0)
		goto out;

	ret = fru_board_area(buf, &board, &end);
	if (ret == -ENODATA || ret == -ENOENT) {
		out[0] = '\0';		/* not provisioned yet -> empty */
		ret = 0;
		goto out;
	}
	if (ret)
		goto out;

	p = board + 6;			/* skip fmt, len, lang, 3-byte date */
	for (i = 0; i < field; i++) {
		int n = fru_decode(p, end, out, outsz);

		if (n < 0) {
			ret = n;
			goto out;
		}
		if (n == 0) {		/* hit end marker early -> empty */
			out[0] = '\0';
			ret = 0;
			goto out;
		}
		p += n;
	}
	ret = fru_decode(p, end, out, outsz);
	if (ret < 0)
		goto out;
	ret = 0;
out:
	kfree(buf);
	return ret;
}

/*
 * Find the MultiRecord Area and its exact byte length by walking the record
 * list (each record's checksums are position-independent, so the block can be
 * relocated verbatim when the board area resizes).
 */
static int fru_multirecord(const u8 *buf, const u8 **mr, size_t *mr_len)
{
	u8 off = buf[5];
	size_t pos, start;

	*mr = NULL;
	*mr_len = 0;
	if (!off)
		return 0;
	start = pos = (size_t)off * 8;
	while (pos + 5 <= FRU_EEPROM_SIZE) {
		u8 fmt = buf[pos + 1];
		size_t rec = 5 + buf[pos + 2];

		if (pos + rec > FRU_EEPROM_SIZE)
			return -EINVAL;
		pos += rec;
		if (fmt & 0x80) {	/* end-of-list bit */
			*mr = buf + start;
			*mr_len = pos - start;
			return 0;
		}
	}
	return -EINVAL;
}

/*
 * Write the FRU image to the EEPROM one page at a time.
 *
 * The FMC EEPROM (at24) on this Xilinx AXI-IIC controller drops multi-page
 * bulk writes: only the first 8-byte page commits before the device enters
 * its ~5ms write cycle and silently NACKs the remaining pages.  Write one
 * page at a time, wait out the write cycle, and verify each page (retrying)
 * so the whole image lands reliably.
 */
static int fru_eeprom_write(struct panther_fru *st, const u8 *buf, size_t len)
{
	size_t off;

	for (off = 0; off < len; off += FRU_PAGE_SIZE) {
		size_t n = min_t(size_t, FRU_PAGE_SIZE, len - off);
		u8 rb[FRU_PAGE_SIZE];
		int tries, ret;

		for (tries = 0; tries < 8; tries++) {
			ret = nvmem_device_write(st->nvmem, off, n,
						 (void *)(buf + off));
			if (ret < 0)
				return ret;
			usleep_range(6000, 8000);	/* EEPROM write cycle */
			ret = nvmem_device_read(st->nvmem, off, n, rb);
			if (ret < 0)
				return ret;
			if (!memcmp(rb, buf + off, n))
				break;			/* page committed */
		}
		if (tries == 8)
			return -EIO;
	}
	return 0;
}

/*
 * Emit one Board Info string field.  Normally plain 8-bit ASCII, but a
 * single-character ASCII field would encode as type/length byte 0xC1, which
 * is indistinguishable from the FRU "no more fields" end marker and would
 * truncate the record.  Such fields are stored as 6-bit ASCII (type/length
 * 0x81) instead, which round-trips through this driver and fru-dump.
 * Returns bytes written, or -EINVAL if a single char is not 6-bit encodable.
 */
static int fru_emit_field(u8 *dst, const char *s, int len)
{
	if (len == 1) {
		int c = (u8)s[0];

		if (c < 0x20 || c > 0x5F)	/* 6-bit ASCII covers 0x20..0x5F */
			return -EINVAL;		/* e.g. lowercase: use >= 2 chars */
		dst[0] = (FRU_TYPE_SIXBIT << 6) | 1;
		dst[1] = c - 0x20;
		return 2;
	}
	dst[0] = (FRU_TYPE_ASCII << 6) | (len & 0x3F);
	memcpy(dst + 1, s, len);
	return len + 1;
}

/*
 * Rebuild the FRU image with one Board Info string field replaced, preserving
 * the other standard fields, any Board Area custom fields, and the MultiRecord
 * Area.  Internal/Chassis/Product areas are unsupported and rejected.
 */
static int fru_write_field(struct panther_fru *st, enum fru_field field,
			   const char *val)
{
	u8 *old = NULL, *new = NULL;
	char fields[FRU_NUM_STD_FIELDS][FRU_STR_MAX + 1];
	const u8 *board, *end, *p, *mr;
	u8 custom[128];
	size_t mr_len, clen = 0;
	u32 mfg_date = 0;
	int i, f, ret, vlen;

	vlen = strlen(val);
	if (vlen > FRU_STR_MAX)
		return -EINVAL;
	for (i = 0; i < vlen; i++)		/* Board Area must be printable */
		if (val[i] < 0x20 || val[i] == 0x7F)
			return -EINVAL;

	old = kmalloc(FRU_EEPROM_SIZE, GFP_KERNEL);
	new = kzalloc(FRU_EEPROM_SIZE, GFP_KERNEL);
	if (!old || !new) {
		ret = -ENOMEM;
		goto out;
	}

	ret = nvmem_device_read(st->nvmem, 0, FRU_EEPROM_SIZE, old);
	if (ret < 0)
		goto out;

	for (i = 0; i < FRU_NUM_STD_FIELDS; i++)
		fields[i][0] = '\0';

	/* Reject areas we do not know how to preserve. */
	if (old[0] == 0x01 && fru_sum(old, 8) == 0 &&
	    (old[1] || old[2] || old[4])) {
		ret = -EOPNOTSUPP;	/* Internal/Chassis/Product present */
		goto out;
	}

	ret = fru_board_area(old, &board, &end);
	if (!ret) {
		mfg_date = board[3] | (board[4] << 8) | (board[5] << 16);
		p = board + 6;
		for (i = 0; i < FRU_NUM_STD_FIELDS; i++) {
			int n = fru_decode(p, end, fields[i], sizeof(fields[i]));

			if (n < 0) {
				ret = n;
				goto out;
			}
			if (n == 0)	/* early end marker: rest stay empty */
				break;
			p += n;
		}
		/* Preserve any custom fields verbatim up to the end marker. */
		while (p < end && *p != FRU_ENDMARK &&
		       clen + FRU_TL_LEN(*p) + 1 <= sizeof(custom)) {
			size_t n = FRU_TL_LEN(*p) + 1;

			memcpy(custom + clen, p, n);
			clen += n;
			p += n;
		}
	} else if (ret != -ENODATA && ret != -ENOENT) {
		goto out;		/* corrupt header: refuse to write */
	}

	/* Apply the change. */
	strscpy(fields[field], val, sizeof(fields[field]));

	ret = fru_multirecord(old, &mr, &mr_len);
	if (ret)
		goto out;

	/* --- assemble the new image ------------------------------------ */
	new[0] = 0x01;			/* common header format version */
	i = 8;

	/* Board Info Area at offset 8. */
	new[3] = 1;
	new[8]  = 0x01;			/* board area format version */
	new[10] = 25;			/* language code: English */
	new[11] = mfg_date & 0xFF;
	new[12] = (mfg_date >> 8) & 0xFF;
	new[13] = (mfg_date >> 16) & 0xFF;
	i = 14;
	for (f = 0; f < FRU_NUM_STD_FIELDS; f++) {
		int n = fru_emit_field(new + i, fields[f], strlen(fields[f]));

		if (n < 0) {
			ret = n;
			goto out;
		}
		i += n;
	}
	if (clen) {			/* custom fields (verbatim) */
		memcpy(new + i, custom, clen);
		i += clen;
	}
	new[i++] = FRU_ENDMARK;
	i = (((i >> 3) + 1) << 3) - 1;	/* pad; i -> checksum byte index */
	if (i >= FRU_EEPROM_SIZE) {
		ret = -ENOSPC;
		goto out;
	}
	new[9] = (i - 8) / 8 + 1;	/* board area length, 8-byte units */
	new[i] = (u8)-fru_sum(new + 8, i - 8);
	i++;

	/* MultiRecord Area, relocated verbatim. */
	if (mr_len) {
		if (i + mr_len > FRU_EEPROM_SIZE) {
			ret = -ENOSPC;
			goto out;
		}
		new[5] = i / 8;
		memcpy(new + i, mr, mr_len);
		i += mr_len;
	}

	new[7] = (u8)-fru_sum(new, 7);	/* common header checksum */

	ret = fru_eeprom_write(st, new, i);
out:
	kfree(old);
	kfree(new);
	return ret;
}

/* -------------------------------------------------------------------------
 * IIO device attributes
 * ------------------------------------------------------------------------- */
static ssize_t fru_attr_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct panther_fru *st = iio_priv(indio_dev);
	int field = to_iio_dev_attr(attr)->address;
	char val[FRU_STR_MAX + 1];
	int ret;

	mutex_lock(&st->lock);
	ret = fru_read_field(st, field, val, sizeof(val));
	mutex_unlock(&st->lock);
	if (ret)
		return ret;
	return sysfs_emit(buf, "%s\n", val);
}

static ssize_t fru_attr_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct panther_fru *st = iio_priv(indio_dev);
	int field = to_iio_dev_attr(attr)->address;
	char val[FRU_STR_MAX + 1];
	int ret;

	strscpy(val, buf, sizeof(val));
	ret = strlen(val);
	while (ret > 0 && (val[ret - 1] == '\n' || val[ret - 1] == '\r'))
		val[--ret] = '\0';

	mutex_lock(&st->lock);
	ret = fru_write_field(st, field, val);
	mutex_unlock(&st->lock);
	return ret ? ret : len;
}

/* -------------------------------------------------------------------------
 * Network identity: ip_address / mac_address
 *
 * Hosted here rather than on ad5791 because this device already carries board
 * identity, and the client reaches it over libiio -- it cannot write /sys.
 *
 * READS return the LIVE interface state through in-kernel APIs, so a readback
 * reflects what the interface actually has, not what was last requested.
 *
 * WRITES are validated strictly here, then handed to a userspace helper using
 * the same call_usermodehelper pattern as synch.c's reboot attribute.
 *
 * UMH_WAIT_EXEC, not UMH_NO_WAIT. call_usermodehelper stores the argv POINTER
 * rather than deep-copying it, so with NO_WAIT the asynchronous exec happens
 * after this function's argv[] array and the caller's value buffer -- both on
 * the stack -- have gone out of scope. That exec'd garbage and failed silently,
 * which is exactly why rebootNow gets away with UMH_WAIT_PROC. WAIT_EXEC waits
 * only for the execve (sub-millisecond), keeping the stack valid, and still
 * returns long before the helper's own settle delay reconfigures eth0. Do not
 * change this to NO_WAIT without heap-allocating argv and the value.
 *
 * WRITING ip_address ALSO SETS THE MAC, derived from the IP (see
 * panther_derive_mac). One write, one reconnect, and a cloned card never keeps
 * the original board's MAC by accident. Writing mac_address on its own is
 * unchanged and sets only the MAC, for when a specific one is wanted.
 *
 * Validation is not cosmetic: the helper runs as root. in4_pton / mac_pton do
 * the parsing, is_valid_ether_addr rejects multicast and all-zero MACs, and
 * obviously-wrong IPv4 (0.x, loopback, multicast/reserved) is refused so a
 * typo cannot strand the board.
 * ------------------------------------------------------------------------- */
#define PANTHER_NETIF   "eth0"
#define PANTHER_NETCFG  "/usr/local/bin/panther-netcfg"

static int panther_netcfg_run(const char *action, const char *v1,
			      const char *v2)
{
	char *argv[6];
	char *envp[] = { "HOME=/", "PATH=/sbin:/bin:/usr/sbin:/usr/bin", NULL };

	argv[0] = "/bin/bash";
	argv[1] = PANTHER_NETCFG;
	argv[2] = (char *)action;
	argv[3] = (char *)v1;
	argv[4] = (char *)v2;   /* NULL terminates the list when unused */
	argv[5] = NULL;

	return call_usermodehelper(argv[0], argv, envp, UMH_WAIT_EXEC);
}

/*
 * Derive the MAC that goes with a new IP, so ONE write changes both and the
 * link drops ONCE. Writing the two separately would cost two reconnects, and
 * the MAC has to change anyway -- a cloned card that keeps the original's MAC
 * puts two identical addresses on the same segment.
 *
 * Only the last byte moves; the first five (OUI and board prefix) are taken
 * from the live interface, which keeps the address unicast and locally correct.
 *
 * Convention: the last two DECIMAL digits of the host octet become the last two
 * HEX digits of the MAC, so the two read identically -- .8 -> ...:08,
 * .22 -> ...:22, .254 -> ...:54. That is a display convention, not arithmetic:
 * 0x22 is not 22. It is chosen so a board's IP can be read off its MAC and vice
 * versa. Octets above 99 keep only their last two digits, which is what makes
 * the mapping many-to-one -- .154 and .254 both yield ...:54.
 */
static int panther_derive_mac(u8 host_octet, char *out, size_t outsz)
{
	struct net_device *nd;
	u8 mac[ETH_ALEN];
	unsigned int two;

	nd = dev_get_by_name(&init_net, PANTHER_NETIF);
	if (!nd)
		return -ENODEV;
	memcpy(mac, nd->dev_addr, ETH_ALEN);
	dev_put(nd);

	two = host_octet % 100;
	mac[ETH_ALEN - 1] = ((two / 10) << 4) | (two % 10);

	if (!is_valid_ether_addr(mac))
		return -EINVAL;

	scnprintf(out, outsz, "%pM", mac);
	return 0;
}

/* Strip trailing newline/CR/space in place, returning the new length. */
static int panther_trim(char *p)
{
	int n = strlen(p);

	while (n > 0 && (p[n - 1] == '\n' || p[n - 1] == '\r' || p[n - 1] == ' '))
		p[--n] = '\0';
	return n;
}

static ssize_t fru_ip_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct net_device *nd;
	struct in_device *in_dev;
	const struct in_ifaddr *ifa;
	int n = 0;

	nd = dev_get_by_name(&init_net, PANTHER_NETIF);
	if (!nd)
		return sysfs_emit(buf, "no-" PANTHER_NETIF "\n");

	/*
	 * List EVERY address, not just the first. Reporting only the first made
	 * this attribute lie: after a change that left a stale address behind,
	 * it returned the old one as though it were the configured value. If
	 * more than one appears here, that is the fault -- not the report.
	 */
	rcu_read_lock();
	in_dev = __in_dev_get_rcu(nd);
	if (in_dev) {
		in_dev_for_each_ifa_rcu(ifa, in_dev) {
			n += sysfs_emit_at(buf, n, "%s%pI4/%d", n ? " " : "",
					   &ifa->ifa_address, ifa->ifa_prefixlen);
		}
	}
	rcu_read_unlock();
	dev_put(nd);

	if (!n)
		return sysfs_emit(buf, "unconfigured\n");
	n += sysfs_emit_at(buf, n, "\n");
	return n;
}

static ssize_t fru_ip_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t len)
{
	char in[64], norm[32], macstr[24];
	const char *end = NULL;
	unsigned int plen = 24;
	u8 ip[4];
	int n, ret;

	strscpy(in, buf, sizeof(in));
	n = panther_trim(in);
	if (!n)
		return -EINVAL;

	if (!in4_pton(in, n, ip, '/', &end))
		return -EINVAL;

	if (end && *end == '/') {
		if (kstrtouint(end + 1, 10, &plen) || plen == 0 || plen > 32)
			return -EINVAL;
	} else if (end && *end != '\0') {
		return -EINVAL;
	}

	/* Refuse 0.x, loopback and multicast/reserved -- a typo here strands the
	 * board behind a serial console. */
	if (ip[0] == 0 || ip[0] == 127 || ip[0] >= 224)
		return -EINVAL;

	scnprintf(norm, sizeof(norm), "%pI4/%u", ip, plen);

	/* Derive the matching MAC and apply both together: one nmcli
	 * transaction, one reactivation, one dropped connection. */
	ret = panther_derive_mac(ip[3], macstr, sizeof(macstr));
	if (ret) {
		pr_err("panther_fru: ip_address %s NOT applied, cannot derive MAC (%d)\n",
		       norm, ret);
		return ret;
	}

	ret = panther_netcfg_run("ipmac", norm, macstr);
	if (ret < 0) {
		pr_err("panther_fru: ip_address %s NOT applied, helper exec failed (%d)\n",
		       norm, ret);
		return ret;
	}

	pr_info("panther_fru: ip_address -> %s, mac_address -> %s (applying via %s)\n",
		norm, macstr, PANTHER_NETCFG);
	return len;
}

static ssize_t fru_mac_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct net_device *nd;
	ssize_t r;

	nd = dev_get_by_name(&init_net, PANTHER_NETIF);
	if (!nd)
		return sysfs_emit(buf, "no-" PANTHER_NETIF "\n");

	r = sysfs_emit(buf, "%pM\n", nd->dev_addr);
	dev_put(nd);
	return r;
}

static ssize_t fru_mac_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t len)
{
	char in[64], norm[24];
	u8 mac[ETH_ALEN];
	int ret;

	strscpy(in, buf, sizeof(in));
	if (!panther_trim(in))
		return -EINVAL;

	if (!mac_pton(in, mac))
		return -EINVAL;
	/* rejects multicast, broadcast and all-zero */
	if (!is_valid_ether_addr(mac))
		return -EINVAL;

	scnprintf(norm, sizeof(norm), "%pM", mac);

	ret = panther_netcfg_run("mac", norm, NULL);
	if (ret < 0) {
		pr_err("panther_fru: mac_address %s NOT applied, helper exec failed (%d)\n",
		       norm, ret);
		return ret;
	}

	pr_info("panther_fru: mac_address -> %s (applying via %s)\n",
		norm, PANTHER_NETCFG);
	return len;
}

static IIO_DEVICE_ATTR(ip_address, 0644, fru_ip_show, fru_ip_store, 0);
static IIO_DEVICE_ATTR(mac_address, 0644, fru_mac_show, fru_mac_store, 0);

static IIO_DEVICE_ATTR(manufacturer, 0644,
		       fru_attr_show, fru_attr_store, FRU_MANUFACTURER);
static IIO_DEVICE_ATTR(product_name, 0644,
		       fru_attr_show, fru_attr_store, FRU_PRODUCT_NAME);
static IIO_DEVICE_ATTR(serial_number, 0644,
		       fru_attr_show, fru_attr_store, FRU_SERIAL_NUMBER);
static IIO_DEVICE_ATTR(board_revision, 0644,
		       fru_attr_show, fru_attr_store, FRU_PART_NUMBER);

static struct attribute *panther_fru_attributes[] = {
	&iio_dev_attr_manufacturer.dev_attr.attr,
	&iio_dev_attr_product_name.dev_attr.attr,
	&iio_dev_attr_serial_number.dev_attr.attr,
	&iio_dev_attr_board_revision.dev_attr.attr,
	&iio_dev_attr_ip_address.dev_attr.attr,
	&iio_dev_attr_mac_address.dev_attr.attr,
	NULL,
};

static const struct attribute_group panther_fru_attr_group = {
	.attrs = panther_fru_attributes,
};

static const struct iio_info panther_fru_info = {
	.attrs = &panther_fru_attr_group,
};

/* -------------------------------------------------------------------------
 * Platform driver probe / remove
 * ------------------------------------------------------------------------- */
static int panther_fru_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct panther_fru *st;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	mutex_init(&st->lock);

	st->nvmem = devm_nvmem_device_get(&pdev->dev, "fru");
	if (IS_ERR(st->nvmem))
		return dev_err_probe(&pdev->dev, PTR_ERR(st->nvmem),
				     "failed to get FRU nvmem\n");

	indio_dev->name = "panther_fru";
	indio_dev->info = &panther_fru_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	dev_info(&pdev->dev, "Panther FMC FRU ready\n");
	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static const struct of_device_id panther_fru_of_match[] = {
	{ .compatible = "sd,fru" },
	{ }
};
MODULE_DEVICE_TABLE(of, panther_fru_of_match);

static struct platform_driver panther_fru_driver = {
	.probe = panther_fru_probe,
	.driver = {
		.name = "panther_fru",
		.of_match_table = panther_fru_of_match,
	},
};
module_platform_driver(panther_fru_driver);

MODULE_AUTHOR("Scott Colson <colsons@sd-star.com>");
MODULE_DESCRIPTION("SD Panther FMC FRU IIO platform driver");
MODULE_LICENSE("GPL v2");
