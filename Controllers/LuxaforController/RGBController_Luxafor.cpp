/*---------------------------------------------------------*\
| RGBController_Luxafor.cpp                                 |
|                                                           |
|   RGBController for Luxafor devices                       |
|                                                           |
|   Adam Honse (calcprogrammer1@gmail.com)      05 Sep 2024 |
|                                                           |
|   This file is part of the OpenRGB project                |
|   SPDX-License-Identifier: GPL-2.0-only                   |
\*---------------------------------------------------------*/

#include "RGBController_Luxafor.h"

RGBController_Luxafor::RGBController_Luxafor(LuxaforController* controller_ptr)
{
    controller          = controller_ptr;

    name                = "Luxafor Device";
    type                = DEVICE_TYPE_ACCESSORY;
    vendor              = "Luxafor";
    description         = "Luxafor Device";
    location            = controller->GetDeviceLocation();
    serial              = controller->GetSerialString();

    mode Direct;
    Direct.name         = "Direct";
    Direct.value        = LUXAFOR_MODE_DIRECT;
    Direct.flags        = MODE_FLAG_HAS_PER_LED_COLOR;
    Direct.color_mode   = MODE_COLORS_PER_LED;
    modes.push_back(Direct);

    // mode Fade;
    // Fade.name           = "Fade";
    // Fade.value          = LUXAFOR_MODE_FADE;
    // Fade.flags          = MODE_FLAG_HAS_PER_LED_COLOR;
    // Fade.color_mode     = MODE_COLORS_PER_LED;
    // modes.push_back(Fade);

    // mode Strobe;
    // Strobe.name         = "Strobe";
    // Strobe.value        = LUXAFOR_MODE_STROBE;
    // Strobe.flags        = MODE_FLAG_HAS_MODE_SPECIFIC_COLOR;
    // Strobe.color_mode   = MODE_COLORS_MODE_SPECIFIC;
    // Strobe.colors_min   = 1;
    // Strobe.colors_max   = 1;
    // Strobe.colors.resize(1);
    // modes.push_back(Strobe);

    // mode Wave;
    // Wave.name           = "Wave";
    // Wave.value          = LUXAFOR_MODE_WAVE;
    // Wave.flags          = MODE_FLAG_HAS_MODE_SPECIFIC_COLOR;
    // Wave.color_mode     = MODE_COLORS_MODE_SPECIFIC;
    // Wave.colors_min     = 1;
    // Wave.colors_max     = 1;
    // Wave.colors.resize(1);
    // modes.push_back(Wave);

    SetupZones();
}

RGBController_Luxafor::~RGBController_Luxafor()
{

}

void RGBController_Luxafor::SetupZones()
{
    /*-----------------------------------------------------*\
    | The Luxafor Flag has 2 zones                          |
    |   * Flag (3 LEDs)                                     |
    |   * Rear (3 LEDs)                                     |
    | The LED index starts at 1. Sending 255 for the LED ID |
    | sets all LEDs at once.                                |
    \*-----------------------------------------------------*/
    unsigned int led_value  = LUXAFOR_LED_FIRST;

    zone flag_zone;
    flag_zone.name          = "Flag";
    flag_zone.type          = ZONE_TYPE_SINGLE;
    flag_zone.leds_min      = 3;
    flag_zone.leds_max      = 3;
    flag_zone.leds_count    = 3;
    flag_zone.matrix_map    = NULL;
    zones.push_back(flag_zone);

    for(std::size_t led_idx = 0; led_idx < flag_zone.leds_count; led_idx++)
    {
        led luxafor_led;
        luxafor_led.name        = "Flag LED";
        luxafor_led.value       = led_value;
        leds.push_back(luxafor_led);

        led_value++;
    }

    zone rear_zone;
    rear_zone.name          = "Rear";
    rear_zone.type          = ZONE_TYPE_SINGLE;
    rear_zone.leds_min      = 3;
    rear_zone.leds_max      = 3;
    rear_zone.leds_count    = 3;
    rear_zone.matrix_map    = NULL;
    zones.push_back(rear_zone);

    for(std::size_t led_idx = 0; led_idx < rear_zone.leds_count; led_idx++)
    {
        led luxafor_led;
        luxafor_led.name        = "Rear LED";
        luxafor_led.value       = led_value;
        leds.push_back(luxafor_led);

        led_value++;
    }

    SetupColors();
}

void RGBController_Luxafor::ResizeZone(int zone, int new_size)
{
    /*-----------------------------------------------------*\
    | This device does not support resizing zones           |
    \*-----------------------------------------------------*/
}

void RGBController_Luxafor::DeviceUpdateLEDs()
{
    for(std::size_t zone_idx = 0; zone_idx < zones.size(); zone_idx++)
    {
        UpdateZoneLEDs(zone_idx);
    }
}

void RGBController_Luxafor::UpdateZoneLEDs(int zone)
{
    for(std::size_t led_idx = 0; led_idx < zones[zone].leds_count; led_idx++)
    {
        UpdateSingleLED(zones[zone].start_idx + led_idx);
    }
}

void RGBController_Luxafor::UpdateSingleLED(int led)
{
    if(modes[active_mode].color_mode == MODE_COLORS_PER_LED)
    {
        unsigned char red = RGBGetRValue(colors[led]);
        unsigned char grn = RGBGetGValue(colors[led]);
        unsigned char blu = RGBGetBValue(colors[led]);

        controller->SendPacket(modes[active_mode].value, leds[led].value, red, grn, blu);
    }
}

void RGBController_Luxafor::DeviceUpdateMode()
{
    switch(modes[active_mode].color_mode)
    {
        case MODE_COLORS_PER_LED:
            DeviceUpdateLEDs();
            break;

        case MODE_COLORS_MODE_SPECIFIC:
            {
            unsigned char red = RGBGetRValue(colors[modes[active_mode].colors[0]]);
            unsigned char grn = RGBGetGValue(colors[modes[active_mode].colors[0]]);
            unsigned char blu = RGBGetBValue(colors[modes[active_mode].colors[0]]);

            controller->SendPacket(modes[active_mode].value, LUXAFOR_LED_ALL, red, grn, blu);
            }
            break;
    }
}

void RGBController_Luxafor::DeviceSaveMode()
{
    /*-----------------------------------------------------*\
    | This device does not support saving                   |
    \*-----------------------------------------------------*/
}
