#!/bin/sh
#
# Run a small fx3lafw sustained-streaming matrix with robust short-read checks.

set -u

CLI=${FX3_SIGROK_CLI:-sigrok-cli}
DRIVER=${FX3_DRIVER:-fx3lafw}
SECONDS_PER_CASE=${FX3_SECONDS:-10}
CASES=${FX3_CASES:-"64M:32 48M:32 96M:16 192M:8"}
RUN_DIR=${FX3_RUN_DIR:-/tmp/fx3lafw-stress-$(date +%Y%m%d-%H%M%S)}
CPU_JOBS=${FX3_CPU_JOBS:-0}
DEBUG_PORT=${FX3_DEBUG_PORT:-}
TRIGGER=${FX3_TRIGGER:-}

mkdir -p "$RUN_DIR"

rate_hz()
{
	case "$1" in
		*M) echo $((${1%M} * 1000000)) ;;
		*m) echo $((${1%m} * 1000000)) ;;
		*K) echo $((${1%K} * 1000)) ;;
		*k) echo $((${1%k} * 1000)) ;;
		*) echo "$1" ;;
	esac
}

channels_for_width()
{
	last=$(($1 - 1))
	channels=
	i=0
	while [ "$i" -le "$last" ]; do
		if [ -n "$channels" ]; then
			channels=$channels,
		fi
		channels=${channels}D$i
		i=$((i + 1))
	done
	echo "$channels"
}

print_rate()
{
	awk -v bps="$1" 'BEGIN { printf "%.1f", bps / 1000000 }'
}

print_pct()
{
	awk -v bps="$1" -v base="$2" 'BEGIN { printf "%.1f", 100 * bps / base }'
}

stress_pids=
serial_pid=

cleanup()
{
	if [ -n "$stress_pids" ]; then
		kill $stress_pids 2>/dev/null || true
	fi
	if [ -n "$serial_pid" ]; then
		kill "$serial_pid" 2>/dev/null || true
	fi
}

trap cleanup EXIT INT TERM

if [ "$CPU_JOBS" -gt 0 ]; then
	i=0
	while [ "$i" -lt "$CPU_JOBS" ]; do
		yes >/dev/null &
		stress_pids="$stress_pids $!"
		i=$((i + 1))
	done
fi

if [ -n "$DEBUG_PORT" ]; then
	if stty -f "$DEBUG_PORT" 115200 raw -echo 2>/dev/null; then
		cat "$DEBUG_PORT" >"$RUN_DIR/debug-uart.log" &
		serial_pid=$!
	else
		echo "WARN: unable to configure debug port $DEBUG_PORT" >&2
	fi
fi

printf 'case\tmode\tsamples/s\tbits\tMB/s\tUSB3-effective%%\tUSB3-raw%%\tseconds\tresult\telapsed\tsent/expected\tlog\n'

for case_spec in $CASES; do
	rate=${case_spec%%:*}
	bits=${case_spec#*:}
	hz=$(rate_hz "$rate")
	samples=$((hz * SECONDS_PER_CASE))
	bytes_per_sec=$((hz * bits / 8))
	channels=$(channels_for_width "$bits")
	mode=stream
	log="$RUN_DIR/${rate}_${bits}bit_${SECONDS_PER_CASE}s.log"

	start=$(date +%s)
	if [ -n "$TRIGGER" ]; then
		mode=trigger
		"$CLI" -d "$DRIVER" -c samplerate="$rate" --samples "$samples" \
			-C "$channels" -t "$TRIGGER" -w -O binary -o /dev/null \
			>"$log" 2>&1
	else
		"$CLI" -d "$DRIVER" -c samplerate="$rate" --samples "$samples" \
			-C "$channels" -O binary -o /dev/null >"$log" 2>&1
	fi
	rc=$?
	elapsed=$(($(date +%s) - start))

	sent=$(sed -n 's/Device only sent \([0-9][0-9]*\) samples\./\1/p' \
		"$log" | tail -1)
	if [ "$rc" -eq 0 ] && [ -z "$sent" ]; then
		result=PASS
		sent=$samples
	else
		result=FAIL
		if [ -z "$sent" ]; then
			sent=unknown
		fi
	fi

	printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s/%s\t%s\n' \
		"$case_spec" "$mode" "$hz" "$bits" \
		"$(print_rate "$bytes_per_sec")" \
		"$(print_pct "$bytes_per_sec" 500000000)" \
		"$(print_pct "$bytes_per_sec" 625000000)" \
		"$SECONDS_PER_CASE" "$result" "$elapsed" "$sent" "$samples" "$log"
done

echo "RUN_DIR=$RUN_DIR"
