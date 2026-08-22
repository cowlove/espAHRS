#pragma once
#include <LovyanGFX.hpp>
class LGFX:public lgfx::LGFX_Device{
  lgfx::Panel_ST7789 panel;
  lgfx::Bus_Parallel8 bus;
public:
  LGFX(){
    auto b=bus.config();
    b.freq_write=20000000;b.freq_read=16000000;
    b.pin_wr=8;b.pin_rd=9;b.pin_d0=39;b.pin_d1=40;b.pin_d2=41;b.pin_d3=42;
    b.pin_d4=45;b.pin_d5=46;b.pin_d6=47;b.pin_d7=48;
    b.pin_rs=7;
    bus.config(b);panel.setBus(&bus);
    auto p=panel.config();
    p.pin_cs=6;p.pin_rst=5;p.pin_busy=-1;
    p.memory_width=240;p.memory_height=320;p.panel_width=170;p.panel_height=320;
    p.offset_x=35;p.offset_y=0;p.offset_rotation=0;
    p.readable=false;p.invert=true;p.rgb_order=false;p.dlen_16bit=false;p.bus_shared=false;
    panel.config(p);setPanel(&panel);
  }
};
