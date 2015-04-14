-- SLCD.VHD (a peripheral module for SCOMP)
-- 2009.10.10
--
-- The simple LCD controller displays a single 16 bit register on the top line
--   of the LCD.
-- It sends an initialization string to the LCD, then repeatedly writes a four-
--   digit hex value to a fixed location in the display.  The value is latched
--   whenever the device is selected by CS.
-- See datasheets for the HD44780 or equivalent LCD controller.  

LIBRARY IEEE;
LIBRARY LPM;

USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.STD_LOGIC_ARITH.ALL;
USE IEEE.STD_LOGIC_UNSIGNED.ALL;
USE LPM.LPM_COMPONENTS.ALL;


ENTITY SLCD IS
  PORT(
    CLOCK_10KHZ : IN  STD_LOGIC;
    RESETN      : IN  STD_LOGIC;
    CS          : IN  STD_LOGIC;
    IO_DATA     : IN  STD_LOGIC_VECTOR(15 DOWNTO 0);
    IO_ADDR     : IN  STD_LOGIC_VECTOR(2 DOWNTO 0);
    LCD_RS      : OUT STD_LOGIC;
    LCD_RW      : OUT STD_LOGIC;
    LCD_E       : OUT STD_LOGIC;
    LCD_D       : OUT STD_LOGIC_VECTOR(7 DOWNTO 0)
  );
END SLCD;


ARCHITECTURE a OF SLCD IS
  TYPE STATE_TYPE IS (
    RESET,
    INIT,
    INIT_CLOCK,
    CURPOS,
    CURPOS_CLOCK,
    NEXTLINE,
    NEXTLINE_CLOCK,
    SWRITE,
    SWRITE_CLOCK
  );
  
  TYPE LCD_DATA_TYPE IS ARRAY (0 TO 7) OF STD_LOGIC_VECTOR(15 DOWNTO 0);
  TYPE CSTR15_TYPE IS ARRAY (0 TO 15) OF STD_LOGIC_VECTOR(7 DOWNTO 0);
  TYPE CSTR08_TYPE IS ARRAY (0 TO  7) OF STD_LOGIC_VECTOR(7 DOWNTO 0);
  TYPE CSTR04_TYPE IS ARRAY (0 TO  31) OF STD_LOGIC_VECTOR(7 DOWNTO 0);

  SIGNAL state   : STATE_TYPE;
  SIGNAL lcd_data : LCD_DATA_TYPE;
  SIGNAL ascii   : CSTR15_TYPE;
  SIGNAL cstr    : CSTR04_TYPE;
  SIGNAL istr    : CSTR08_TYPE;
  SIGNAL count   : INTEGER RANGE 0 TO 1000;
  SIGNAL delay   : INTEGER RANGE 0 TO 100;


  BEGIN
    -- LCD initialization string
    istr(0) <= x"38"; -- Wakeup
    istr(1) <= x"38"; -- Wakeup
    istr(2) <= x"38"; -- Wakeup
    istr(3) <= x"38"; -- Function set: 2 lines, 5x8 dot font
    istr(4) <= x"08"; -- Display off
    istr(5) <= x"01"; -- Clear display
    istr(6) <= x"0C"; -- Display on
    istr(7) <= x"04"; -- Entry mode set (left to right)

    ascii( 0) <= x"30"; -- ASCII table values
    ascii( 1) <= x"31";
    ascii( 2) <= x"32";
    ascii( 3) <= x"33";
    ascii( 4) <= x"34";
    ascii( 5) <= x"35";
    ascii( 6) <= x"36";
    ascii( 7) <= x"37";
    ascii( 8) <= x"38";
    ascii( 9) <= x"39";
    ascii(10) <= x"41";
    ascii(11) <= x"42";
    ascii(12) <= x"43";
    ascii(13) <= x"44";
    ascii(14) <= x"45";
    ascii(15) <= x"46";

    LCD_RW  <= '0';
    cstr(28) <= ascii(CONV_INTEGER(lcd_data(0)( 3 DOWNTO  0)));
    cstr(29) <= ascii(CONV_INTEGER(lcd_data(0)( 7 DOWNTO  4)));
    cstr(30) <= ascii(CONV_INTEGER(lcd_data(0)( 11 DOWNTO  8)));
    cstr(31) <= ascii(CONV_INTEGER(lcd_data(0)( 15 DOWNTO  12)));
    
    cstr(24) <= ascii(CONV_INTEGER(lcd_data(1)( 3 DOWNTO  0)));
    cstr(25) <= ascii(CONV_INTEGER(lcd_data(1)( 7 DOWNTO  4)));
    cstr(26) <= ascii(CONV_INTEGER(lcd_data(1)( 11 DOWNTO  8)));
    cstr(27) <= ascii(CONV_INTEGER(lcd_data(1)( 15 DOWNTO  12)));
    
    cstr(20) <= ascii(CONV_INTEGER(lcd_data(2)( 3 DOWNTO  0)));
    cstr(21) <= ascii(CONV_INTEGER(lcd_data(2)( 7 DOWNTO  4)));
    cstr(22) <= ascii(CONV_INTEGER(lcd_data(2)( 11 DOWNTO  8)));
    cstr(23) <= ascii(CONV_INTEGER(lcd_data(2)( 15 DOWNTO  12)));
    
    cstr(16) <= ascii(CONV_INTEGER(lcd_data(3)( 3 DOWNTO  0)));
    cstr(17) <= ascii(CONV_INTEGER(lcd_data(3)( 7 DOWNTO  4)));
    cstr(18) <= ascii(CONV_INTEGER(lcd_data(3)( 11 DOWNTO  8)));
    cstr(19) <= ascii(CONV_INTEGER(lcd_data(3)( 15 DOWNTO  12)));
    
    cstr(12) <= ascii(CONV_INTEGER(lcd_data(4)( 3 DOWNTO  0)));
    cstr(13) <= ascii(CONV_INTEGER(lcd_data(4)( 7 DOWNTO  4)));
    cstr(14) <= ascii(CONV_INTEGER(lcd_data(4)( 11 DOWNTO  8)));
    cstr(15) <= ascii(CONV_INTEGER(lcd_data(4)( 15 DOWNTO  12)));
    
    cstr(8) <= ascii(CONV_INTEGER(lcd_data(5)( 3 DOWNTO  0)));
    cstr(9) <= ascii(CONV_INTEGER(lcd_data(5)( 7 DOWNTO  4)));
    cstr(10) <= ascii(CONV_INTEGER(lcd_data(5)( 11 DOWNTO  8)));
    cstr(11) <= ascii(CONV_INTEGER(lcd_data(5)( 15 DOWNTO  12)));
    
    cstr(4) <= ascii(CONV_INTEGER(lcd_data(6)( 3 DOWNTO  0)));
    cstr(5) <= ascii(CONV_INTEGER(lcd_data(6)( 7 DOWNTO  4)));
    cstr(6) <= ascii(CONV_INTEGER(lcd_data(6)( 11 DOWNTO  8)));
    cstr(7) <= ascii(CONV_INTEGER(lcd_data(6)( 15 DOWNTO  12)));
    
    cstr(0) <= ascii(CONV_INTEGER(lcd_data(7)( 3 DOWNTO  0)));
    cstr(1) <= ascii(CONV_INTEGER(lcd_data(7)( 7 DOWNTO  4)));
    cstr(2) <= ascii(CONV_INTEGER(lcd_data(7)( 11 DOWNTO  8)));
    cstr(3) <= ascii(CONV_INTEGER(lcd_data(7)( 15 DOWNTO  12)));

    -- This process latches the incoming data value on the rising edge of CS
    PROCESS (RESETN, CS)
      BEGIN
        IF (RESETN = '0') THEN
          lcd_data(0) <= x"0000";
          lcd_data(1) <= x"0000";
          lcd_data(2) <= x"0000";
          lcd_data(3) <= x"0000";
          lcd_data(4) <= x"0000";
          lcd_data(5) <= x"0000";
          lcd_data(6) <= x"0000";
          lcd_data(7) <= x"0000";
        ELSIF (RISING_EDGE(CS)) THEN
          lcd_data(CONV_INTEGER(IO_ADDR)) <= IO_DATA;
        END IF;
      END PROCESS;


    -- This processes writes the latched data values to the LCD
    PROCESS (RESETN, CLOCK_10KHZ)
      BEGIN
        IF (RESETN = '0') THEN
          LCD_D  <= x"00";
          LCD_RS <= '0';
          LCD_E  <= '0';
          count  <= 0;
          delay  <= 0;
          state  <= RESET;

        ELSIF (RISING_EDGE(CLOCK_10KHZ)) THEN
          CASE state IS
            WHEN RESET =>           -- wait about 0.1 sec (exceeds 15 ms requirement)
              IF (count > 999) THEN
                count <= 0;
                state <= INIT;
              ELSE
                count <= count + 1;
              END IF;

            WHEN INIT =>            -- send an init command
              LCD_RS <= '0';
              LCD_E  <= '1';
              LCD_D  <= istr(count);
              count  <= count + 1;
              delay  <= 0;
              state  <= INIT_CLOCK;

            WHEN INIT_CLOCK =>      -- latch the command and wait
              LCD_E <= '0';         --  dropping LCD_E latches
              delay <= delay + 1;

              IF (delay >= 99) THEN  -- wait about 10 ms between init commands
                IF (count < 8) THEN
                  state <= INIT;
                ELSE
                  state <= CURPOS;
                END IF;
              END IF;

            -- all remaining states have no waits.  100 us per state
            -- write (enable) states alternate with latching states

            WHEN CURPOS =>
              LCD_RS <= '0';
              LCD_E  <= '1';
			  LCD_D  <= x"CF";
              state  <= CURPOS_CLOCK;

            WHEN CURPOS_CLOCK =>
              LCD_E <= '0';
              count <= 0;
              state <= SWRITE;

            WHEN NEXTLINE =>
              LCD_RS <= '0';
              LCD_E  <= '1';
			  LCD_D  <= x"8F";
              state  <= NEXTLINE_CLOCK;

            WHEN NEXTLINE_CLOCK =>
              LCD_E <= '0';
              state <= SWRITE;

            WHEN SWRITE =>            -- Write (least significant digit first)
              LCD_RS <= '1';
              LCD_E  <= '1';
              LCD_D  <= cstr(count);
              count  <= count + 1;
              state <= SWRITE_CLOCK;

            WHEN SWRITE_CLOCK =>      -- Finish write (moves left on screen in chosen mode)
              LCD_E <= '0';
              IF (count >= 32) THEN
                state <= CURPOS;
              ELSIF (count = 16) THEN
                state <= NEXTLINE;
              ELSE
                state <= SWRITE;
              END IF;

          END CASE;
        END IF;
      END PROCESS;
  END a;

