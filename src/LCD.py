'''
Created on Dec 21, 2015

@author: mac
'''
from bbio import *

KEYPAD = {(1,0):'1',(2,0):'2',(4,0):'3',(8,0):'4',(16,0):'5',(32,0):'6',
          (64,0):'7',(128,0):'8',(0,1):'9',(0,4):'0',(0,2):'*',(0,8):'#'}

class LCD():
    def __init__(self,address):
        self.ad = address
        self.open()
        self.backlight_on()
        I2C2.write(self.ad,[0,27,128,128,140,144,151,143,135,133,130])
        I2C2.write(self.ad,[0,27,129,128,148,156,159,159,158,138,149])
        I2C2.write(self.ad,[0,27,130,128,140,144,151,143,135,133,138])
        I2C2.write(self.ad,[0,27,131,128,148,156,159,159,158,138,148])
        self.clear_screen()
        self.write_text('Initalise')
        self.hide_cursor()
        
        
        
    def backlight_on(self):
        I2C2.write(self.ad, [0,19])
    
    def clear_screen(self):
        I2C2.write(self.ad, [0,12])
    
    def home_cursor(self):
        I2C2.write(self.ad, [0,1])
        
    def set_cursor(self,line,col):
        I2C2.write(self.ad, [0,3,line,col])
    
    def hide_cursor(self):
        I2C2.write(self.ad, [0,4])
        
    def show_underline_cursor(self):
        I2C2.write(self.ad, [0,5])
    
    def show_blink_cursor(self):
        I2C2.write(self.ad, [0,6])
            
    def write_text(self,text):
        asc_t = [ord(c) for c in text]
        I2C2.write(self.ad, [0] + asc_t)
    
    def draw_1(self):
        I2C2.write(self.ad,[0,128])
        I2C2.write(self.ad,[0,129])
    def draw_2(self):
        I2C2.write(self.ad,[0,130])
        I2C2.write(self.ad,[0,131])
                
    def keypad_input(self):
        c = I2C2.read(self.ad, 3)
        while c[1] == 0 and c[2] == 0:
            c = I2C2.read(self.ad, 3)
        text_tuple = (c[1],c[2])    
        return KEYPAD[text_tuple]
    
    def sapce_tab(self, n = 3):
        I2C2.write(self.ad,[0,n])
        
        
    def open(self):
        I2C2.open()
        
    def close(self):
        I2C2.close()
    
    
    
        
        
    
    
        