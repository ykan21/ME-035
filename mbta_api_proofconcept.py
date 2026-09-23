import network
import urequests
import ujson
import time
import ntptime
import gc

# 1. Hotspot Credentials
WIFI_SSID = "ゆーのIphone" 
WIFI_PASS = "iphone21"

# MBTA Endpoint: Checks predictions sorted by arrival_time first, then departure_time
MBTA_URL = "https://api-v3.mbta.com/predictions?filter[stop]=place-mdftf&sort=arrival_time,departure_time&page[limit]=3"

# Eastern Daylight Time (EDT) offset from UTC = -4 hours (-14400 seconds)
EDT_OFFSET = -14400  

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    
    # Reset Wi-Fi interface to prevent internal state errors
    wlan.active(False)
    time.sleep(0.5)
    wlan.active(True)
    time.sleep(0.5)

    if not wlan.isconnected():
        print("Connecting to Hotspot...")
        wlan.connect(WIFI_SSID, WIFI_PASS)
        
        timeout = 20
        start = time.time()
        while not wlan.isconnected():
            if time.time() - start > timeout:
                print("\n[Error] Connection timed out! Check SSID or Password.")
                return False
            time.sleep(0.5)
            print(".", end="")
            
    print("\nConnected! IP Address:", wlan.ifconfig()[0])
    return True

def parse_iso_to_utc_seconds(iso_str):
    """Parses ISO string (e.g. '2026-09-23T17:15:00-04:00') to UTC epoch seconds."""
    date_part, time_part = iso_str.split("T")
    year, month, day = map(int, date_part.split("-"))
    
    if "-" in time_part[8:]:
        clean_time, tz_str = time_part.split("-", 1)
        tz_sign = -1
    elif "+" in time_part[8:]:
        clean_time, tz_str = time_part.split("+", 1)
        tz_sign = 1
    else:
        clean_time = time_part.rstrip("Z")
        tz_str = "00:00"

    hour, minute, second = map(int, clean_time.split(":"))
    
    tz_h, tz_m = map(int, tz_str.split(":")) if ":" in tz_str else (0, 0)
    offset_seconds = tz_sign * ((tz_h * 3600) + (tz_m * 60))
    
    raw_epoch = time.mktime((year, month, day, hour, minute, second, 0, 0))
    return raw_epoch - offset_seconds

def fetch_next_departure():
    """Queries MBTA API for the immediate next train arriving or departing at Tufts/Medford."""
    try:
        response = urequests.get(MBTA_URL)
        if response.status_code == 200:
            data = response.json()
            response.close()
            
            predictions = data.get("data", [])
            now_utc = time.time()
            
            for pred in predictions:
                attrs = pred["attributes"]
                # Check arrival_time first to catch incoming trains, fallback to departure_time
                iso_time = attrs.get("arrival_time") or attrs.get("departure_time")
                
                if iso_time:
                    target_utc = parse_iso_to_utc_seconds(iso_time)
                    # Pick the first prediction occurring in the future
                    if target_utc > now_utc:
                        gc.collect()
                        return target_utc
        else:
            response.close()
    except Exception as e:
        print("\n[API Error]:", e)
        
    gc.collect()
    return None

def format_local_clock_time(utc_epoch):
    """Converts UTC epoch seconds to local 12-hour clock time (e.g. '5:15 PM')."""
    if not utc_epoch:
        return None
    
    local_epoch = utc_epoch + EDT_OFFSET
    tm = time.localtime(local_epoch)
    
    hour = tm[3]
    minute = tm[4]
    second = tm[5]
    
    ampm = "AM" if hour < 12 else "PM"
    hour_12 = hour % 12 or 12
        
    return "{}:{:02d}:{:02d} {}".format(hour_12, minute, second, ampm)

# --- MAIN RUNTIME ---

if connect_wifi():
    print("Syncing internal clock with NTP server...")
    try:
        ntptime.settime()
        print("Clock synced successfully!")
    except Exception as e:
        print("NTP sync failed:", e)

    target_utc_epoch = None
    last_api_fetch = 0

    print("\n--- Live MBTA Tufts/Medford Tracker Active ---")

    while True:
        current_time = time.time()
        
        # Poll API every 15 seconds to catch live updates
        if current_time - last_api_fetch > 15 or target_utc_epoch is None:
            print("\nFetching next train...")
            new_target = fetch_next_departure()
            if new_target:
                target_utc_epoch = new_target
            last_api_fetch = current_time

        if target_utc_epoch:
            clock_str = format_local_clock_time(target_utc_epoch)
            seconds_remaining = target_utc_epoch - time.time()
            
            if seconds_remaining > 0:
                print("Next Train at Tufts/Medford: {} (in {}s)     ".format(clock_str, seconds_remaining), end="\r")
            else:
                print("Train departing now!                                     ", end="\r")
                target_utc_epoch = None  # Force immediate query for the next train
        else:
            print("Waiting for train prediction data...                      ", end="\r")

        time.sleep(1)