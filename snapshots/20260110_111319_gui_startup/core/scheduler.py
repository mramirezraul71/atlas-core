# -*- coding: utf-8 -*-
import schedule
import time
from core.logger import log

def heartbeat():
    log("ATLAS heartbeat OK")

def run_scheduler():
    schedule.every(10).seconds.do(heartbeat)
    while True:
        schedule.run_pending()
        time.sleep(1)
