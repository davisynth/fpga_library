--This file is part of the Davisynth FPGA Library
--Copyright (C) 2021  Davisynth
--
--This program is free software: you can redistribute it and/or modify
--it under the terms of the GNU General Public License as published by
--the Free Software Foundation, either version 3 of the License, or
--(at your option) any later version.
--
--This program is distributed in the hope that it will be useful,
--but WITHOUT ANY WARRANTY; without even the implied warranty of
--MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--GNU General Public License for more details.
--
--You should have received a copy of the GNU General Public License
--along with this program.  If not, see <http://www.gnu.org/licenses/>.


--The radiant I2C and SPI IPs suck. Here's a better interface with comments
--VHDL 2008

--IMPORTANT!!!
--Not tested on open source tools, but you should have a thing called SB_I2C (similar to the spi stuff)
--In radiant, make an I2C module from the IP catalog. Call it SB_I2C 


LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
use IEEE.numeric_std.all;
use IEEE.numeric_std_unsigned.all;
library iCE40UP;
use iCE40UP.components.all;

ENTITY hard_i2c_controller IS
	GENERIC(
		max_writes : natural := 1;
		max_reads : natural := 1;
		clock_divider : std_logic_vector(7 downto 0) := "00111100" --For 48MHz
	);
	PORT( 
		clk  : in std_logic; 
		reset : in std_logic; --synchronous
        device_address : in std_logic_vector(6 downto 0); --The address of the device you're trying to use (1st byte in i2c diagrams)
		write_data : in std_logic_vector(8*max_writes-1 downto 0); --8 byte write register
		read_data : out std_logic_vector(8*max_reads-1 downto 0); --8 byte read register
		n_writes : in integer range 0 to max_writes; --Number of writes you want to make
		n_reads : in integer range 0 to max_reads; --Number of reads you want to make
		do_transmission : in std_logic; --Hold this high until finished=1 to transmit you data. Then you can blip it low to do a new transmission. 
		finished : out std_logic; --Will be high if finished until a new transmission starts
        sda : inout std_logic;
        scl : inout std_logic
	);
END hard_i2c_controller;

ARCHITECTURE synth OF hard_i2c_controller IS
    --Uncomment this for Open source tools. Lattice sucks. 
	--component SB_I2C is
	--port(
	--	SBCLKi : in std_logic;
	--	SBWRi  : in std_logic;
	--	SBSTBi : in std_logic;
	--	SBADRi : in std_logic_vector(7 downto 0);
	--	SBDATi : in std_logic_vector(7 downto 0);
	--	SBDATo : out std_logic_vector(7 downto 0);
	--	SBACKo : out std_logic;
	--	I2CPIRQ  	: out std_logic;
	--	I2CPWKUP	: out std_logic;
	--	I2C1_SCL	: inout std_logic;
	--	I2C1_SDA1	: inout std_logic -- Might be I2C1_SDA1
	--);
	--end component;

    component SB_I2C is
        port(
        	SB_CLK_I : in std_logic;
        	SB_WR_I  : in std_logic;
        	SB_STB_I : in std_logic;
        	SB_ADR_I : in std_logic_vector(7 downto 0);
        	SB_DAT_I : in std_logic_vector(7 downto 0);
        	SB_DAT_O : out std_logic_vector(7 downto 0);
        	SB_ACK_O : out std_logic;
        	I2C_PIRQ_O  : out std_logic_vector(1 downto 0);
        	I2C_PWKUP_O	: out std_logic_vector(1 downto 0);
        	I2C1_SCL_IO	: inout std_logic;
        	I2C1_SDA_IO	: inout std_logic -- Might be I2C1_SDA1
        );
        end component;    

    --Useful to have open alongside this file:
	--AdvancediCE40SPII2CHardenedIPUsageGuide.pdf
	--Congtrol registers and stuff are from that

	TYPE I2C_States IS (INIT_I2CCR1, INIT_I2CBRLSB, INIT_I2CBRMSB, WAIT_FOR_CMD, TX_CHIP_SET_ADDR,TX_CHIP_SET_CMD,TX_CHECK_STATUS, TX_TRANSMIT_SET_DATA, TX_TRANSMIT_SEND, RX_CHIP_SET_ADDR, RX_CHIP_SET_CMD, RX_CHIP_CHECK_STATUS, RX_RECEIVE_SEND, RX_CHECK_STATUS, RX_RECEIVE_SET_DATA, RX_END, RX_END_SEND_STOP, RX_END_CHECK_STATUS, RX_END_LAST_READ, STOP);  -- Define the states
	SIGNAL state_i2c : I2C_States;
	


	constant I2C_CR1 : std_logic_vector     := "00000001";
	constant I2C_BRLSB : std_logic_vector   := "00000010";
    constant I2C_BRMSB : std_logic_vector   := "00000011";
    constant I2C_CMDR : std_logic_vector    := "00000111";
    constant I2C_TXDR : std_logic_vector    := "00001000";
    constant I2C_RXDR : std_logic_vector    := "00001001";
    constant I2C_SR : std_logic_vector      := "00001011";
	
	signal send_Counter : integer range 0 to max_writes := 0;
    signal receive_Counter : integer range 0 to max_reads := 0;

    signal i2c_rw : std_logic := '0';
    signal i2c_stb : std_logic := '0';
    signal i2c_ack : std_logic := '0';
    signal i2c_addr : std_logic_vector(7 downto 0) := "00000000";
    signal i2c_datai : std_logic_vector(7 downto 0) := "00000000";
    signal i2c_datao : std_logic_vector(7 downto 0) := "00000000";

    signal wait_bit : std_logic := '0';

BEGIN
    --Uncomment this for Open source tools. Lattice sucks. 
    --hard_i2c_inst : SB_I2C
    --port map(
    --    SBCLKi      => clk,             --: in std_logic;
	--	SBWRi       => i2c_rw,             --: in std_logic;
	--	SBSTBi      => i2c_stb,             --: in std_logic;
	--	SBADRi      => i2c_addr,    --: in std_logic_vector(7 downto 0);
	--	SBDATi      => i2c_datai,   --: in std_logic_vector(7 downto 0);
	--	SBDATo      => i2c_datao,             --: out std_logic_vector(7 downto 0);
	--	SBACKo      => i2c_ack,             --: out std_logic;
	--	I2CPIRQ     => open,             --: out std_logic;
	--	I2CPWKUP    => open,             --: out std_logic;
	--	I2C1_SCL    => scl,             --: inout std_logic;
	--	I2C1_SDA1    => sda             --: inout std_logic;
    --);

    hard_i2c_inst : SB_I2C
    port map(
        SB_CLK_I      => clk,             --: in std_logic;
		SB_WR_I       => i2c_rw,             --: in std_logic;
		SB_STB_I      => i2c_stb,             --: in std_logic;
		SB_ADR_I      => i2c_addr,    --: in std_logic_vector(7 downto 0);
		SB_DAT_I      => i2c_datai,   --: in std_logic_vector(7 downto 0);
		SB_DAT_O      => i2c_datao,             --: out std_logic_vector(7 downto 0);
		SB_ACK_O      => i2c_ack,             --: out std_logic;
		I2C_PIRQ_O    => open,             --: out std_logic;
		I2C_PWKUP_O   => open,             --: out std_logic;
		I2C1_SCL_IO   => scl,             --: inout std_logic;
		I2C1_SDA_IO   => sda             --: inout std_logic;
    );

	process(clk)
	begin
		if rising_edge(clk) then
			case state_i2c is
				--Enable I2C core
				when INIT_I2CCR1 =>
					i2c_addr <= I2C_CR1;
					i2c_datai <= "10000000";
					i2c_stb <= '1';
					i2c_rw <= '1'; --write
					if i2c_ack = '1' then
						 i2c_stb <= '0';
						 state_i2c <= INIT_I2CBRLSB;
					end if;
				--The System Bus clock frequency is divided by (I2C_PRESCALE*4) 
				--to produce the Master I2C clock frequency supported by the I2C 
				--bus (50 kHz, 100 kHz, 400 kHz).
				--For 48MHz input clock, we want 200kHz -> 240x/4 = 60x = 0b00111100
				when INIT_I2CBRLSB =>
					i2c_addr <= I2C_BRLSB;
					i2c_datai <= clock_divider;					
					i2c_stb <= '1';
					i2c_rw <= '1'; --write
					if i2c_ack = '1' then
						 i2c_stb <= '0';
						 state_i2c <= INIT_I2CBRMSB;
					end if;
				--Set MSB register even if unused. Idk if this does anything. 
				when INIT_I2CBRMSB =>
					i2c_addr <= I2C_BRMSB;
					i2c_datai <= "00000000";					
					i2c_stb <= '1';
					i2c_rw <= '1'; --write
					if i2c_ack = '1' then
						 i2c_stb <= '0';
						 state_i2c <= WAIT_FOR_CMD;
					end if;
				when WAIT_FOR_CMD =>
					state_i2c <= TX_CHIP_SET_ADDR;
				--Start TX
				--Set device address and say to write (this is the first byte you see in i2c diagrams)
				when TX_CHIP_SET_ADDR =>
					i2c_addr <= I2C_TXDR;
					i2c_datai <= device_address&'1'; --Device addr + W
					i2c_stb <= '1';
					i2c_rw <= '1'; --write
					if i2c_ack = '1' then
						i2c_stb <= '0';
						state_i2c <= TX_CHIP_SET_CMD;
				   end if;
				when TX_CHIP_SET_CMD =>
				   	i2c_addr <= I2C_CMDR;
				   	i2c_datai <= "10010100"; --7=Start, 4=Write to peripheral, 2=Disable clock stretching
				   	i2c_stb <= '1';
					i2c_rw <= '1'; --write
				   	if i2c_ack = '1' then
						i2c_stb <= '0';
					   	state_i2c <= TX_CHECK_STATUS;
						send_counter <= n_writes;
				  	end if;
				--Basically we have to make sure the peripheral acknowledged our i2c "hello"
				when TX_CHECK_STATUS =>
					i2c_addr <= I2C_SR;
					i2c_stb <= '1';
					i2c_rw <= '0'; --read
					if i2c_ack = '1' then
					 	i2c_stb <= '0';
						if(i2c_datao(2) = '1') then --If ready to transmit, do it
							state_i2c <= TX_TRANSMIT_SET_DATA;
							if(i2c_datao(5) = '0') then
								state_i2c <= TX_CHIP_SET_ADDR; --If NACK, gotta start over :/
							end if;
						end if;
			   		end if;
				when TX_TRANSMIT_SET_DATA => 				
					if send_counter = 0 then
						if n_reads > 0 then
							state_i2c <= RX_CHIP_SET_ADDR;
						else
							state_i2c <= STOP;
						end if;
					else
						i2c_addr <= I2C_TXDR;
						i2c_datai <= write_data(7+8*(send_counter-1)downto 8*(send_counter-1)); --Send correct data (order is msBYTE to lsBYTE, data is read left to right)
						i2c_stb <= '1';
						i2c_rw <= '1'; --write
						if i2c_ack = '1' then
							i2c_stb <= '0';
							state_i2c <= TX_TRANSMIT_SEND;
						end if;
					end if; 
                when TX_TRANSMIT_SEND =>
                    i2c_addr <= I2C_CMDR;
                    i2c_datai <= "00010100"; --4=Write to peripheral, 2=Disable clock stretching
                    i2c_stb <= '1';
                    i2c_rw <= '1'; --write
                    if i2c_ack = '1' then
                        i2c_stb <= '0';
                        state_i2c <= TX_CHECK_STATUS;
                        send_counter <= send_counter - 1;
                    end if;
                --This is literallty the same as the TX stuff, but for RX
				when RX_CHIP_SET_ADDR =>
                    i2c_addr <= I2C_TXDR;
                    i2c_datai <= device_address&'0'; --Device addr + R
                    i2c_stb <= '1';
                    i2c_rw <= '1'; --write
                    if i2c_ack = '1' then
                        i2c_stb <= '0';
                        state_i2c <= RX_CHIP_SET_CMD;
                    end if;
                when RX_CHIP_SET_CMD =>
                    i2c_addr <= I2C_CMDR;
                    i2c_datai <= "10010100"; --7=Start, 4=Write to peripheral, 2=Disable clock stretching
                    i2c_stb <= '1';
                    i2c_rw <= '1'; --write
                    if i2c_ack = '1' then
                     i2c_stb <= '0';
                     state_i2c <= RX_CHECK_STATUS;
                     receive_counter <= n_reads;
                   end if;
                when RX_CHIP_CHECK_STATUS =>
                    i2c_addr <= I2C_SR;
                    i2c_stb <= '1';
                    i2c_rw <= '0'; --read
                    if i2c_ack = '1' then
                        i2c_stb <= '0';
                        if(i2c_datao(4) = '1') then --WAIT FOR SRW
                            state_i2c <= RX_RECEIVE_SEND;
                            if(i2c_datao(5) = '0') then
                                state_i2c <= RX_CHIP_SET_ADDR; --If NACK, gotta start over :/
                            end if;
                        end if;
                    end if;
                when RX_RECEIVE_SEND => 				
					if receive_counter <= 1 then
						state_i2c <= RX_END;
                        wait_bit <= '0'; --Reset the wait bit
					else
						i2c_addr <= I2C_CMDR;
						i2c_datai <= "00100100"; --5=Read from peripheral, 2=Disable clock stretching
						i2c_stb <= '1';
						i2c_rw <= '1'; --write
						if i2c_ack = '1' then
							i2c_stb <= '0';
							state_i2c <= RX_CHECK_STATUS;
							receive_counter <= receive_counter - 1;
						end if;
					end if; 
                when RX_CHECK_STATUS =>
                    i2c_addr <= I2C_SR;
                    i2c_stb <= '1';
                    i2c_rw <= '0'; --read
                    if i2c_ack = '1' then
                        i2c_stb <= '0';
                        if(i2c_datao(2) = '1') then --If ready to recieve, do it
                            state_i2c <= RX_RECEIVE_SET_DATA;
                            if(i2c_datao(5) = '0') then
                                state_i2c <= RX_CHIP_SET_ADDR; --If NACK, gotta start over :/
                            end if;
                        end if;
                    end if;
                when RX_RECEIVE_SET_DATA =>
                    i2c_addr <= I2C_RXDR; --Look at our received data register to get the data 
                    i2c_stb <= '1';
                    i2c_rw <= '0'; --read
                    if i2c_ack = '1' then
                        i2c_stb <= '0';
                        read_data(7+8*(receive_counter-1)downto 8*(receive_counter-1)) <= i2c_datao;
                        state_i2c <= RX_RECEIVE_SEND;
                    end if;
                --Gotta wait 2 clock cycles
                when RX_END =>
                    if wait_bit = '1' then
                        state_i2c <= RX_END_SEND_STOP;
                    else
                        wait_bit <= '1';
                    end if;
                when RX_END_SEND_STOP =>
                    i2c_addr <= I2C_CMDR;
                    i2c_datai <= "01101100"; --6=Stop, 5=Read from peripheral, 3=Send NACK, 2=Disable clock stretching
                    i2c_stb <= '1';
                    i2c_rw <= '1'; --write
                    if i2c_ack = '1' then
                        i2c_stb <= '0';
                        state_i2c <= RX_END_CHECK_STATUS;
                    end if;
                when RX_END_CHECK_STATUS =>
                    i2c_addr <= I2C_SR;
                    i2c_stb <= '1';
                    i2c_rw <= '0'; --read
                    if i2c_ack = '1' then
                        i2c_stb <= '0';
                        if(i2c_datao(2) = '1') then --Wait for TRReady
                            state_i2c <= RX_END_LAST_READ; 
                        end if;
                        receive_counter <= n_reads;
                    end if;
                when RX_END_LAST_READ =>
                    i2c_addr <= I2C_RXDR; --Look at our received data register to get the data 
                    i2c_stb <= '1';
                    i2c_rw <= '0'; --read
                    if i2c_ack = '1' then
                        i2c_stb <= '0';
                        read_data(7 downto 0) <= i2c_datao(7 downto 0);
                        state_i2c <= WAIT_FOR_CMD;
                    end if;
                when STOP =>
                    i2c_addr <= I2C_CMDR;
                    i2c_datai <= "01000100"; --6=Stop, 2=Disable clock stretching
                    i2c_stb <= '1';
                    i2c_rw <= '1'; --write
                    if i2c_ack = '1' then
                        i2c_stb <= '0';
                        state_i2c <= WAIT_FOR_CMD;
                    end if;
				when others => null;
			end case;
		end if;
	end process;
	
END synth;