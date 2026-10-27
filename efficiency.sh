#!/bin/bash
# Efficiency characterisation of the protocol (Data-Link Protocol guide, slide 31).
#
# Runs tx/rx over the virtual cable, varying one parameter at a time around a
# baseline, and writes S = R/C to a CSV file:
#   R = 8 * file size / transfer time (llOpen -> llClose, from the Tx statistics)
#   C = cable baud rate
# Note: the cable sends 10 bits per byte (8-N-1), so S can never exceed ~0.8.
#
# Usage: ./efficiency.sh            (from the project directory, needs socat)
# Every list below can be overridden from the environment, e.g.:
#   RUNS=3 BAUDS="9600 38400" ./efficiency.sh

set -u
cd "$(dirname "$0")"

FILE=${FILE:-penguin.gif}
OUT=${OUT:-efficiency_results.csv}
RUNS=${RUNS:-1}                          # repetitions of each configuration

# Baseline (used for every parameter that is not being swept)
BASE_BAUD=${BASE_BAUD:-38400}
BASE_SIZE=${BASE_SIZE:-997}              # file bytes per data packet
BASE_FER=${BASE_FER:-0}                  # BCC2 (data field) error probability
BASE_DELAY=${BASE_DELAY:-0}              # receiver processing delay per I frame (us)
HDR_PROB=${HDR_PROB:-0}                  # BCC1 error probability (each one costs a 4 s timeout)

# Values swept
BAUDS=${BAUDS:-"9600 19200 38400 57600 115200"}
SIZES=${SIZES:-"100 250 500 997"}
FERS=${FERS:-"0 0.1 0.2 0.3 0.5"}
DELAYS=${DELAYS:-"0 10000 25000 50000 100000"}

TX_PORT=/tmp/ttyS10
RX_PORT=/tmp/ttyS11

command -v socat > /dev/null || { echo "Error: socat not found"; exit 1; }
[ -f "$FILE" ] || { echo "Error: $FILE not found"; exit 1; }

WORK=$(mktemp -d)
CABLE_CMDS=$WORK/cable_cmds
FILE_SIZE=$(stat -c %s "$FILE")

cleanup()
{
    echo "quit" >> "$CABLE_CMDS"
    sleep 0.5
    kill "$CABLE_PID" 2> /dev/null
    pkill -f "tail -f $CABLE_CMDS" 2> /dev/null
    rm -rf "$WORK"
}

# Start the cable once; commands are sent to it by appending to $CABLE_CMDS
mkdir -p bin
gcc -Wall -o bin/cable cable/cable.c || exit 1
: > "$CABLE_CMDS"
tail -f "$CABLE_CMDS" | ./bin/cable > "$WORK/cable.log" 2>&1 &
CABLE_PID=$!
trap cleanup EXIT
sleep 1.5

echo "param,baud,data_size,fer,hdr_prob,delay_us,run,ok,time_s,R_bps,S,Tf_s,a" > "$OUT"

# run_test <param> <baud> <size> <fer> <delay_us>
run_test()
{
    local param=$1 baud=$2 size=$3 fer=$4 delay=$5
    local bin=$WORK/main_${size}_${fer}_${HDR_PROB}_${delay}

    if [ ! -x "$bin" ]; then
        gcc -Wall -DMAX_DATA_SIZE="$size" -DBCC2_ERROR_PROB="$fer" \
            -DBCC1_ERROR_PROB="$HDR_PROB" -DPROC_DELAY_US="$delay" \
            -o "$bin" src/*.c || exit 1
    fi

    echo "baud $baud" >> "$CABLE_CMDS"
    sleep 0.5

    for run in $(seq 1 "$RUNS"); do
        rm -f "$WORK/received"
        timeout 300 "$bin" "$RX_PORT" "$baud" rx "$WORK/received" > "$WORK/rx.log" 2>&1 &
        local rx_pid=$!
        sleep 0.3
        timeout 300 "$bin" "$TX_PORT" "$baud" tx "$FILE" > "$WORK/tx.log" 2>&1
        wait "$rx_pid"

        local ok=0
        cmp -s "$FILE" "$WORK/received" && ok=1
        local time
        time=$(awk '/Elapsed time/ {print $3}' "$WORK/tx.log")
        [ -z "$time" ] && time=nan

        # Tf: time to send one full I frame (data packet + 3 header + 6 frame bytes, 10 bits each)
        # a = Tprop/Tf, with the receiver delay modelling the round trip (2 * Tprop)
        awk -v p="$param" -v b="$baud" -v k="$size" -v f="$fer" -v h="$HDR_PROB" \
            -v d="$delay" -v r="$run" -v ok="$ok" -v t="$time" -v n="$FILE_SIZE" 'BEGIN {
                tf = (k + 9) * 10 / b
                a = (d / 1e6) / 2 / tf
                if (t == "nan" || t <= 0) { R = "nan"; S = "nan" }
                else { R = 8 * n / t; S = R / b }
                printf "%s,%d,%d,%s,%s,%d,%d,%d,%s,%s,%s,%.6f,%.4f\n",
                       p, b, k, f, h, d, r, ok, t, R, S, tf, a
            }' >> "$OUT"

        printf "%-6s baud=%-6s size=%-4s fer=%-4s delay=%-6s run=%s ok=%s time=%ss\n" \
               "$param" "$baud" "$size" "$fer" "$delay" "$run" "$ok" "$time"
    done
}

for b in $BAUDS;  do run_test baud  "$b" "$BASE_SIZE" "$BASE_FER" "$BASE_DELAY"; done
for k in $SIZES;  do run_test size  "$BASE_BAUD" "$k" "$BASE_FER" "$BASE_DELAY"; done
for f in $FERS;   do run_test fer   "$BASE_BAUD" "$BASE_SIZE" "$f" "$BASE_DELAY"; done
for d in $DELAYS; do run_test delay "$BASE_BAUD" "$BASE_SIZE" "$BASE_FER" "$d"; done

echo "Results written to $OUT"
