#!/usr/bin/env python3

import requests

url = "http://wd1.wspr.rocks/api/clickhouse/"

# data = {"parms": "true", "balloons": "hide", "txCall": "VU3FOE", "band": "all", "hours": 840, "unique": "false", "count": 500}
data = {"parms": "true", "balloons": "hide", "rxCall": "", "txCall": "VU3FOE",
        "band": "all", "hours": 24, "unique": "false", "count": 512,
        "advanced": ""}

r = requests.post(url=url, data=data)

data = r.json()["data"]

for row in data:
    # print(row)
    dbm = row[15]
    callsign = row[7]
    timestamp = row[1]
    if dbm == 30:
        print("[%s] %s ==> %s" % (timestamp, callsign, "I am OK"))
    elif dbm == 33:
        print("[%s] %s ==> %s" % (timestamp, callsign, "I am NOT OK"))
    elif dbm == 37:
        print("[%s] %s ==> %s" % (timestamp, callsign, "Please check on me!"))
