import board
import busio
from adafruit_mcp230xx.mcp23017 import MCP23017
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from tqdm.auto import tqdm


mcp_pin_numbers = {0: [0, 1, 2, 3],
                   1: [4, 5, 6, 7],
                   2: [8, 9, 10, 11],
                   3: [12, 13, 14, 15]}
binary_dictionary = {0: [0, 0, 0, 0],
                     1: [1, 0, 0, 0],
                     2: [0, 1, 0, 0],
                     3: [1, 1, 0, 0],
                     4: [0, 0, 1, 0],
                     5: [1, 0, 1, 0],
                     6: [0, 1, 1, 0],
                     7: [1, 1, 1, 0],
                     8: [0, 0, 0, 1],
                     9: [1, 0, 0, 1],
                     10: [0, 1, 0, 1],
                     11: [1, 1, 0, 1],
                     12: [0, 0, 1, 1],
                     13: [1, 0, 1, 1],
                     14: [0, 1, 1, 1],
                     15: [1, 1, 1, 1]}


def setup_mcp():
    i2c = busio.I2C(board.SCL, board.SDA)
    mcp = MCP23017(i2c)
    _mcp_pins = {0: mcp.get_pin(0), 1: mcp.get_pin(1), 2: mcp.get_pin(2), 3: mcp.get_pin(3),
                 4: mcp.get_pin(4), 5: mcp.get_pin(5), 6: mcp.get_pin(6), 7: mcp.get_pin(7),
                 8: mcp.get_pin(8), 9: mcp.get_pin(9), 10: mcp.get_pin(10), 11: mcp.get_pin(11),
                 12: mcp.get_pin(12), 13: mcp.get_pin(13), 14: mcp.get_pin(14), 15: mcp.get_pin(15)}

    for pin in _mcp_pins:
        _mcp_pins[pin].switch_to_output(value=False)

    return _mcp_pins


def setup_ads():
    i2c = busio.I2C(board.SCL, board.SDA)
    ads = ADS.ADS1115(i2c, address=0x48)
    ads_channels = {0: AnalogIn(ads, ADS.P0), 1: AnalogIn(ads, ADS.P1),
                    2: AnalogIn(ads, ADS.P2), 3: AnalogIn(ads, ADS.P3)}
    return ads_channels


mcp_pins = setup_mcp()
ads_channels = setup_ads()


def set_sensor_pin(sensor_number, ads_channel_number):
    binary_instructions = binary_dictionary[sensor_number]
    mcp_pins_to_change = mcp_pin_numbers[ads_channel_number]
    for mcp_pin_to_change, binary_instruction in zip(mcp_pins_to_change, binary_instructions):
        mcp_pins[mcp_pin_to_change].value = binary_instruction


def get_sensor_value(sensor_number, ads_channel_number):
    set_sensor_pin(sensor_number, ads_channel_number)
    return ads_channels[ads_channel_number].voltage


if __name__ == "__main__":
    for _ in range(100000000):
        a = []
        for ads_channel in range(4):
            for sen_number in range(6):
                a.append(get_sensor_value(sen_number, ads_channel))
        print(a)

@TODO "Write a function that calls all the values of an arm parallely"
