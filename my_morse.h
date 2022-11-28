/* morse table , shift left to transmit, base of the table is a comma */

const unsigned char morse[] PROGMEM = {
0b11001110, 0b10001100, 0b01010110, 0b10010100,                 // comma to slash
0b11111100, 0b01111100, 0b00111100, 0b00011100, 0b00001100,     // 0 to 4
0b00000100, 0b10000100, 0b11000100, 0b11100100, 0b11110100,     // 5 to 9
0,0,0,0,0,      0b00110010,  0,                                 // ? and filler
0b01100000, 0b10001000, 0b10101000, 0b10010000, 0b01000000,     // a to e
0b00101000, 0b11010000, 0b00001000, 0b00100000, 0b01111000,     // f to j
0b10110000, 0b01001000, 0b11100000, 0b10100000, 0b11110000,     // k to o 
0b01101000, 0b11011000, 0b01010000, 0b00010000, 0b11000000,     // p to t
0b00110000, 0b00011000, 0b01110000, 0b10011000, 0b10111000, 0b11001000 };


/*
// lower case a sends shift as an idle code
const unsigned char baudot_table[2][32] = {
{  0,'E','\n','A',' ','S','I','U','\r','D','R','J','N','F','C',
  'K','T','Z','L','W','H','Y','P','Q','O','B','G',0,'M',
  'X','V','a' } ,
  
{ 0,'3','\n','-',' ',0,'8','7','\r','$','4','\'',',','!',
  ':','(','5','"',')','2','#','6','0','1','9','?','&',0,
  '.','/',';',0 }  
};
*/

/*
const unsigned char baudot_letters[] = {
  0,'E','\n','A',' ','S','I','U','\r','D','R','J','N','F','C',
  'K','T','Z','L','W','H','Y','P','Q','O','B','G',0,'M',
  'X','V',0 };
  
const unsigned char baudot_figures[] = {
  0,'3','\n','-',' ',0,'8','7','\r','$','4','\'',',','!',
  ':','(','5','"',')','2','#','6','0','1','9','?','&',0,
  '.','/',';',0 };
*/  
