#!/bin/bash -x

set -e  # exit script on error

# URL for REST server
url="localhost:8080";
curl="curl -s"

echo "* Configuring terminals in ringtopo.py network"
#for i in {1..2}
#do
#  tr="t${i}"
#  for j in {1..20}
#  do
#    curl "$url/connect?node=${tr}&ethPort=40&wdmPort=${j}&channel=1"
#  done
#done
$curl "$url/connect?node=t1&ethPort=30&wdmPort=2&channel=1"
$curl "$url/connect?node=t2&ethPort=30&wdmPort=2&channel=1"

echo "* Monitoring signals at endpoints"
for tr in {1..2}; do
    tname="t${tr}"
    $curl "$url/monitor?monitor=${tname}-monitor"
done

echo "* Resetting ROADM"
for rc in {1..2}; do
  rcname="r${rc}"
  $curl "$url/reset?node=${rcname}"
done

echo "* Configuring ROADM to forward ch1 from t1 to t2"
# for tr in {1..20}; do
#  link=$((tr+2))
  $curl "$url/connect?node=r1&port1=4&port2=2&channels=1"
  # forward signal from r2 to t2
  $curl "$url/connect?node=r2&port1=1&port2=4&channels=1"
# done

echo "* Turning on terminals/transceivers"
for tr in {1..2}; do
  tname="t${tr}"
  $curl "$url/turn_on?node=${tname}"
done

echo "* Monitoring signals at endpoints"
for tr in {1..2}; do
    tname="t${tr}"
    echo "* $tname"
    $curl "$url/monitor?monitor=${tname}-monitor"
done

echo "* Done."
