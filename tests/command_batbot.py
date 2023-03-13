""" PY TO ESP (LED CONTROLLER) """
# Written by Junicchi - https://github.com/Kebablord

import urllib.request

root_url = "http://IP-ADRESS-OF-ESP"  # ESP's url, ex: http://192.168.102 (Esp prints it to serial console when connected to wifi)


def send_request(url):
    n = urllib.request.urlopen(url)  # send request to ESP


# Example usage
while True:
    answer = input(""" To control the led, type "ON" or "OFF": """)
    if (answer == "ON"):
        send_request(root_url + "/OPEN_LED")
        print("Opened!\n\n")
    if (answer == "OFF"):
        send_request(root_url + "/CLOSE_LED")
        print("Closed!\n\n")
