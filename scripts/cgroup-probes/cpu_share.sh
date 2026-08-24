#!/usr/bin/env bash
# Does SCHED_BATCH cost a CPU-BOUND task throughput?
# CFS weight is identical; the only difference is wakeup preemption, which a
# task that never sleeps never uses. So the expectation is zero — measured
# rather than assumed.
set -u
POL=$1; LOAD=$2
for i in $(seq 1 "$LOAD"); do (while :; do :; done) & done
LOADPIDS=$(jobs -p)
sleep 1
# The measured task: fixed work, timed.
( [ "$POL" = batch ] && chrt --batch -p 0 $BASHPID >/dev/null 2>&1
  s=$(date +%s%N)
  awk 'BEGIN{x=1.0; for(i=0;i<40000000;i++) x=x*1.0000001+1.0; }'
  e=$(date +%s%N)
  echo "  $POL  fixed work in $(( (e-s)/1000000 )) ms" ) 
kill $LOADPIDS 2>/dev/null
wait 2>/dev/null
