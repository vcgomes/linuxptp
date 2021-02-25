/**
 * @file sysoff.c
 * @brief Implements the system offset estimation method.
 * @note Copyright (C) 2012 Richard Cochran <richardcochran@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include <stdio.h>
#include <string.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <linux/ptp_clock.h>

#include "print.h"
#include "sysoff.h"

#define NS_PER_SEC 1000000000LL
#define NS_PER_MSEC 1000000LL

static int64_t pctns(struct ptp_clock_time *t)
{
	return t->sec * NS_PER_SEC + t->nsec;
}

static void nspct(int64_t t, struct ptp_clock_time *pct)
{
	pct->sec = t / NS_PER_SEC;
	pct->nsec = t % NS_PER_SEC;
}

static int sysoff_precise(int fd, int64_t *result, uint64_t *ts)
{
	struct ptp_sys_offset_precise pso;
	memset(&pso, 0, sizeof(pso));
	if (ioctl(fd, PTP_SYS_OFFSET_PRECISE, &pso)) {
		pr_debug("ioctl PTP_SYS_OFFSET_PRECISE: %m");
		return SYSOFF_RUN_TIME_MISSING;
	}
	*result = pctns(&pso.sys_realtime) - pctns(&pso.device);
	*ts = pctns(&pso.sys_realtime);
	return SYSOFF_PRECISE;
}

static int sysoff_cross(int fd, int64_t *result, uint64_t *ts, int64_t *delay)
{
	struct pollfd pfd = { fd, POLLIN, 0 };
	struct ptp_extts_event_cross *event = NULL;
	static char buf[4096];
	ssize_t ret;
	int num;

	ret = poll(&pfd, 1, 0);
	if (ret < 1) {
		pr_debug("poll sysoff_cross: %m");
		return SYSOFF_RUN_TIME_MISSING;
	}

	ret = read(fd, buf, sizeof(buf));
	if (ret < 0) {
		pr_debug("read sysoff_cross: %m");
		return SYSOFF_RUN_TIME_MISSING;
	}

	if (ret < sizeof(*event)) {
		pr_debug("invalid event size");
		return SYSOFF_RUN_TIME_MISSING;
	}

	num = ret / sizeof(*event);
	event = (struct ptp_extts_event_cross *) &buf[(num - 1) * sizeof(*event)];

	if (!(event->flags & PTP_EVENT_CROSS)) {
		pr_debug("invalid event flags");
		return SYSOFF_RUN_TIME_MISSING;
	}

	*result = pctns(&event->t) - event->tstamp;
	*ts = pctns(&event->t);
	*delay = PTP_EVENT_CROSS_DELAY(event->flags);

	return SYSOFF_CROSS;
}

static int sysoff_cross_enable(int fd, int period)
{
	struct ptp_crossts_request cross;
	memset(&cross, 0, sizeof(cross));

	nspct(period, &cross.period);
	if (ioctl(fd, PTP_CROSSTS_REQUEST, &cross)) {
		pr_debug("ioctl PTP_CROSSTS_REQUEST: %m");
		return SYSOFF_RUN_TIME_MISSING;
	}

	return SYSOFF_CROSS;
}

static int64_t sysoff_estimate(struct ptp_clock_time *pct, int extended,
			       int n_samples, uint64_t *ts, int64_t *delay)
{
	int64_t t1, t2, tp;
	int64_t interval, timestamp, offset;
	int64_t shortest_interval, best_timestamp, best_offset;
	int i = 0;

	if (extended) {
		t1 = pctns(&pct[3*i]);
		tp = pctns(&pct[3*i+1]);
		t2 = pctns(&pct[3*i+2]);
	} else {
		t1 = pctns(&pct[2*i]);
		tp = pctns(&pct[2*i+1]);
		t2 = pctns(&pct[2*i+2]);
	}
	shortest_interval = t2 - t1;
	best_timestamp = (t2 + t1) / 2;
	best_offset = best_timestamp - tp;

	for (i = 1; i < n_samples; i++) {
		if (extended) {
			t1 = pctns(&pct[3*i]);
			tp = pctns(&pct[3*i+1]);
			t2 = pctns(&pct[3*i+2]);
		} else {
			t1 = pctns(&pct[2*i]);
			tp = pctns(&pct[2*i+1]);
			t2 = pctns(&pct[2*i+2]);
		}
		interval = t2 - t1;
		timestamp = (t2 + t1) / 2;
		offset = timestamp - tp;
		if (interval < shortest_interval) {
			shortest_interval = interval;
			best_timestamp = timestamp;
			best_offset = offset;
		}
	}
	*ts = best_timestamp;
	*delay = shortest_interval;
	return best_offset;
}

static int sysoff_extended(int fd, int n_samples,
			   int64_t *result, uint64_t *ts, int64_t *delay)
{
	struct ptp_sys_offset_extended pso;
	memset(&pso, 0, sizeof(pso));
	pso.n_samples = n_samples;
	if (ioctl(fd, PTP_SYS_OFFSET_EXTENDED, &pso)) {
		pr_debug("ioctl PTP_SYS_OFFSET_EXTENDED: %m");
		return SYSOFF_RUN_TIME_MISSING;
	}
	*result = sysoff_estimate(&pso.ts[0][0], 1, n_samples, ts, delay);
	return SYSOFF_EXTENDED;
}

static int sysoff_basic(int fd, int n_samples,
			int64_t *result, uint64_t *ts, int64_t *delay)
{
	struct ptp_sys_offset pso;
	memset(&pso, 0, sizeof(pso));
	pso.n_samples = n_samples;
	if (ioctl(fd, PTP_SYS_OFFSET, &pso)) {
		perror("ioctl PTP_SYS_OFFSET");
		return SYSOFF_RUN_TIME_MISSING;
	}
	*result = sysoff_estimate(pso.ts, 0, n_samples, ts, delay);
	return SYSOFF_BASIC;
}

int sysoff_measure(int fd, int method, int n_samples,
		   int64_t *result, uint64_t *ts, int64_t *delay)
{
	switch (method) {
	case SYSOFF_CROSS:
		*delay = 0;
		return sysoff_cross(fd, result, ts, delay);
	case SYSOFF_PRECISE:
		*delay = 0;
		return sysoff_precise(fd, result, ts);
	case SYSOFF_EXTENDED:
		return sysoff_extended(fd, n_samples, result, ts, delay);
	case SYSOFF_BASIC:
		return sysoff_basic(fd, n_samples, result, ts, delay);
	}
	return SYSOFF_RUN_TIME_MISSING;
}

int sysoff_probe(int fd, int n_samples)
{
	int64_t junk, delay;
	uint64_t ts;
	int period = 1 * NS_PER_MSEC;
	int i;

	if (n_samples > PTP_MAX_SAMPLES) {
		fprintf(stderr, "warning: %d exceeds kernel max readings %d\n",
			n_samples, PTP_MAX_SAMPLES);
		fprintf(stderr, "falling back to clock_gettime method\n");
		return SYSOFF_RUN_TIME_MISSING;
	}

	if (sysoff_cross_enable(fd, period) >= 0)
		return SYSOFF_CROSS;

	for (i = SYSOFF_CROSS + 1; i < SYSOFF_LAST; i++) {
		if (sysoff_measure(fd, i, n_samples, &junk, &ts, &delay) < 0)
			continue;
		return i;
	}

	return SYSOFF_RUN_TIME_MISSING;
}
