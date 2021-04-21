//***************************************************************
//  ���j�o�[�T���L�b�g 3.0.1 ���C���v���O����
//                                          2016/09/26 H. Suzuki
//   �C�� 2018/7/4 5 12/21 ito �w�������n���p�ɏC��
//
//  2016/09/16  Sample. 01 �F�������݃`�F�b�N�p
//  2016/09/16  Sample. 02 �FLED�_��
//  2016/09/16  Sample. 03 �FLCD�\��
//  2016/09/16  Sample. 04 �FTimer W - D�̂�
//  2016/09/16  Sample. 05 �FSW �ǉ�
//  2016/09/16  Sample. 06 �FMode System �ǉ�
//  2016/09/16  Sample. 07 �FA/D �ǉ�
//  2016/09/16  Sample. 08 �FBeep �ǉ�
//  2016/09/26  Sample. 09 �FSensor �ǉ�
//  2016/09/26  Sample. 10 �FMotor1(�蓮�쓮) �ǉ�
//  2016/09/26  Sample. 11 �FMotor2(���荞�݋쓮) �ǉ�
//  2016/09/26  Sample. 12 �FMotor3(�p������), MotorTest �ǉ�
//  2016/09/26  Sample. 13 �FSTEP, 1&N��ԑO�i �ǉ�
//  2016/09/26  Sample. 14 �FTURN �ǉ�
//  2016/09/26  Sample. 15 �F���H����(�T���֐�) �ǉ�
//  2016/09/26  Sample. 16 �F����@ �ǉ�
//  2016/09/26  Sample. 17 �F���W�X�V �ǉ�
//  2016/09/26  Sample. 18 �F�}�b�s���O �ǉ��C�T���@�֐���
//  2016/09/26  Sample. 19 �F�g������@ �ǉ�
//  2016/09/26  Sample. 20 �F�����@ �ǉ�
//  2016/09/26  Sample. 21 �F�񎟑��s �ǉ�
//
//***************************************************************

#include "3694.h"
#include "140225LCD.h"        // LCD�p�֐��w�b�_
#include "160926AccTable.h"   // ���[�^���x�p�w�b�_

//---------------------------------------------------------------
//  �^�錾
//---------------------------------------------------------------
#define  uchar    unsigned char
#define  ushort   unsigned short
#define  vshort   volatile short
#define  vushort  volatile unsigned short

//-------------------------------------------------------------------------
//  �|�[�g��`
//-------------------------------------------------------------------------
#define   CPU_LED     IO.PDR2.BIT.B0    // CPU�wLED
#define   LED         IO.PDR1.BIT.B0    // LED�X�C�b�`
#define   SW_EXEC     IO.PDR1.BIT.B4    // ���s�X�C�b�`
#define   SW_UP       IO.PDR1.BIT.B5    // ���[�h�I���X�C�b�`+
#define   SW_DOWN     IO.PDR1.BIT.B6    // ���[�h�I���X�C�b�`-
#define   MOTOR_EN    IO.PDR8.BIT.B0    // ���[�^�h���C�o�X�C�b�`
#define   L_MOT_MODE  IO.PDR8.BIT.B3    // �����[�^�̉�]����
#define   R_MOT_MODE  IO.PDR8.BIT.B4    // �E���[�^�̉�]����

//-------------------------------------------------------------------------
//  �}�N����`
//-------------------------------------------------------------------------
// �X�C�b�`�֘A
#define   SW_ON       0    // �X�C�b�`ON (Active Low)
#define   SW_OFF      1    // �X�C�b�`OFF
#define   KEY_OFF   200    // �X�C�b�`�p�`���^�����O�L�����Z������
// ���[�h�֘A
#define   ModeMax     8    // ���샂�[�h��
#define   DISP        0    // ���[�h�\��
#define   EXEC        1    // ���[�h���s
// �Z���T�֘A
#define   LED_ON      1    // �Z���T�pLED�_��
#define   LED_OFF     0    // �Z���T�pLED����
// ���[�^�֘A
#define   LeftGo      1    // �����[�^�O�i
#define   LeftBack    0    // �����[�^��i
#define   RightGo     0    // �E���[�^�O�i
#define   RightBack   1    // �E���[�^��i
// �T���֘A
#define   GOAL_X      3    // �S�[�� x���W  �t���� 7,7 ���� 8,8 
#define   GOAL_Y      5    // �S�[�� y���W 
#define   S_MODE      0    // Search Mode : ���T����Ԃ͕ǖ����Ƃ��Ĉ���
#define   T_MODE      1    // Try Mode    : ���T����Ԃ͕ǗL��Ƃ��Ĉ���

//---------------------------------------------------------------
//  �O���[�o���ϐ���`
//---------------------------------------------------------------
vushort  wait_timer = 0;   // �������v( msec ) : wait�֐��p�J�E���^
ushort   SENSOR_PT;        // ���荞�݉񐔃J�E���g�p�|�C���^
int      MODE = 0;         // ���݃��[�h�i�[�p
vshort   Batt;             // �d�r�̓d��
//�u�U�[�֘A : �x��,�h,�h#,��,��#,�~,�t�@,�t�@#,�\,�\#,��,��#,�V,�h
unsigned char beep_data[14]
 = { 0, 149, 141, 133, 126, 118, 112, 106, 100, 94, 89, 84, 79, 75 };
// �Z���T�̎��O�l�i�[�p
vushort  R_PRE;            // �E�Z���T�̒l
vushort  L_PRE;            // ���Z���T�̒l
vushort  F_PRE;            // �O�Z���T�̒l
// �Z���T�̌��ݒl�i�[�p
vushort  R_SEN;            // �E�Z���T�̒l
vushort  L_SEN;            // ���Z���T�̒l
vushort  F_SEN;            // �O�Z���T�̒l
// �Z���T��ON/OFF�p
short    R_SW;             // �E�Z���T�̃X�C�b�`
short    L_SW;             // ���Z���T�̃X�C�b�`
short    F_SW;             // �O�Z���T�̃X�C�b�`
// �Z���T�̂������l
short    R_REF;            // �E�Z���T�������l
short    L_REF;            // ���Z���T�������l
// �ǂ̗L������p�������l
short    R_LIM;            // �E�ǗL���������l
short    L_LIM;            // ���ǗL���������l
short    F_LIM;            // �O�ǗL���������l
// ���[�^�֘A
ushort   timerL;           // ���^�C�}�[�ݒ�l
ushort   timerR;           // �E�^�C�}�[�ݒ�l
short    ldir;             // �����[�^��]����
short    rdir;             // �E���[�^��]����
short    speed;            // �ڕW���x
short    speed_now;        // ���ݑ��x
short    MotorTimer;       // ���[�^�d���R���g���[���^�C�}�[
short    control_mode;     // �p�����䃂�[�h  0:�Ȃ�  1:����
// ���s�֘A
short    STEP;             // ���[�^�̃X�e�b�v��
short    GO_STEP;          // 1��Ԃ̃X�e�b�v��
short    TURN_STEP;        // ���M����X�e�b�v��
// �T���֘A
uchar    head;             // �}�E�X�̐i�s���� 0:�k 1:�� 2:�� 3:��
uchar    head_change;      // �i�s�����X�V�p�ϐ� 0:�O 1:�E 2:�� 3:��
uchar    pos_x;            // �}�E�X�̌��ݍ��W x
uchar    pos_y;            // �}�E�X�̌��ݍ��W y
uchar    map[16][16];      // MAP�f�[�^
uchar    p_map[16][16];    // �|�e���V����MAP�f�[�^

//---------------------------------------------------------------
//  �֐��v���g�^�C�v�錾
//---------------------------------------------------------------
void IO_init( void );
void load_param( void );
void pause( int x );
void WaitKeyOff( void );
void beep(unsigned char tone,int value);
void change_mode( int x );
void exec_mode( void );
void mode0( int x );
void mode1( int x );
void mode2( int x );
void mode3( int x );
void mode4( int x );
void mode5( int x );
void mode6( int x );

void mode7( int x );

void mouse_search( int goal_x, int goal_y, int speed, int mode );
void com_go( int n );
void com_back( int n);
void com_stop( void );
void com_turn( int t_mode );
void countdown( void );
void finish( void );
int get_wall_data( void );
void clear_map( void );
void make_map_data( void );
void make_potential( int gx, int gy, int mode );
int search_left_hand( void );
int search_ex_left_hand( void );
int search_adachi( void );

//---------------------------------------------------------------
//  ���C���v���O����
//---------------------------------------------------------------
void main( void )
{
  IO_init();    // H8�̏�����
  LCD_init();   // LCD�̏�����
  CPU_LED = 1;  // CPU�wLED������

  // �N����
  beep( 3, 150 );
  beep( 8, 150 );

  // �^�C�g���\��
//  LCD_print( 0, "Sample21" );
  LCD_print( 0, "Samp21b" );  //2018/12/21
  
  // �d���\��
  LCD_print( 8, "   .  v " );
  LCD_dec_out( 9, Batt/100, 1);     // �\�̈ʂ�\��
  Batt %= 100;                      // �\�̈ʂ��폜
  LCD_dec_out(10, Batt/10 , 1);     // ��̈ʂ�\��
  Batt %= 10;                       // ��̈ʂ��폜
  LCD_dec_out(12, Batt    , 1);     // �c���������l��\��
  pause( 2000 );

  clear_map();                     // MAP�f�[�^������
  load_param();                    // �e��p�����[�^��ǂݍ���
  change_mode( 0 );                // �܂�������ʂɂ��� = Mode0

  // ���C�����[�v
  while( 1 ){
    if( SW_UP == SW_ON ){          // ��SW��������Ă���ꍇ
      WaitKeyOff();                // �`���^�����O�h�~����
      change_mode(+1);             // ���[�h+1
    }else if( SW_DOWN == SW_ON ){  // ��SW��������Ă���ꍇ
      WaitKeyOff();                // �`���^�����O�h�~����
      change_mode(-1);             // ���[�h-1
    }else if( SW_EXEC == SW_ON ){  // ���sSW��������Ă���ꍇ
      beep( 1, 150 );              // ���s�� : �h
      WaitKeyOff();                // �`���^�����O�h�~����
      exec_mode();                 // ���[�h���s
      MODE = 0;
      change_mode( 0 );            // ���s��͏�����ʂɖ߂�
    }
    
    // ���[�h��0�Ȃ�Z���T�f�[�^��LCD�\��
    if( MODE == 0 )
    {
      LCD_dec_out(  3, F_SEN, 3 ); // �O�Z���T�l��LCD�㒆���ɕ\��
      LCD_dec_out(  9, L_SEN, 3 ); // ���Z���T�l��LCD�����ɕ\��
      LCD_dec_out( 13, R_SEN, 3 ); // �E�Z���T�l��LCD�E���ɕ\��
    }
  }
}

//---------------------------------------------------------------
//  H8������
//---------------------------------------------------------------
void IO_init( void )
{
  // ===== I/O �|�[�g�̏����� =====           �s���ԍ�  7654 3210
  IO.PCR2 = 0x01;       // P20���o��, ���g�p�͓���      0000 0001
  IO.PCR5 = 0x3f;       // P50-P55���o��, ���g�p�͓���  0011 1111
  IO.PCR1 = 0x01;       // P14-P16�����, P10���o��     0000 0001
  IO.PUCR1.BYTE = 0x70; // P14-16�v���A�b�v 0:OFF/1:ON  0111 0000
  IO.PCR8 = 0x1f;       // P80-P84 ���o�̓|�[�g�ɐݒ�   0001 1111

  // ===== �o�b�e���[�d���v�� =====
  // ===== A/D �ݒ� =====
  // AD.ADCSR.BYTE ���
  // 7:END Flg  6:���荞��(1:ON) 5:�ϊ��X�^�[�g(1) 4:(0)�P��(1)Scan
  // 3:Clock(0)134state / (1)70state<�����ϊ�>
  // 210: �P�ꃂ�[�h�� AN***
  //      scan���[�h�� ��Ɏw�肵��A/D�ϊ�����������
  //                   0:AN0,1:AN0-1,2:AN0-2,3:AN0-3,
  //                   4:AN4,5:AN4-5,6:AN4-6,7:AN4-7
  // 0010 1111 = 0x2f // AN7 �P�ꍂ���ϊ�
  AD.ADCSR.BYTE = 0x2f;             // AN7 �P�ꍂ���ϊ�
  while( AD.ADCSR.BIT.ADF == 0 );   // ���肪�I������܂ő҂�
  Batt = AD.ADDRD >> 7;             // ����f�[�^AN7����荞��

  // ===== �^�C�}�[�̏����� =====
  // Timer V ���荞�ݐݒ�
  TV.TCRV0.BIT.CCLR   = 1;    // �R���y�A�}�b�`A��TCNTV�N���A
  TV.TCRV0.BIT.CKS    = 0;    // �����N���b�N�ݒ� : 0(�ŏ��̓N���b�N�x�~)
  TV.TCSRV.BIT.OS     = 3;    // �R���y�A�}�b�`A��P76(TMOV)�Ƀg�O���o��
  TV.TCORA            = 0;    // ����(����)�̐ݒ背�W�X�^ : 0(�ŏ��͋x��)

  // Timer W ���荞�ݐݒ�
  TW.TIERW.BIT.IMIEA  = 1;   // ���荞��A��L��
  TW.TIERW.BIT.IMIEB  = 1;   // ���荞��B��L��
  TW.TIERW.BIT.IMIEC  = 1;   // ���荞��C��L��
  TW.TIERW.BIT.IMIED  = 1;   // ���荞��D��L��
  TW.TCRW.BIT.CKS     = 1;   // ���荞��Clock�Z���N�g1/2 : 20M/2=10MHz
  TW.TIOR0.BIT.IOA    = 2;   // �R���y�A�}�b�`��FTIOA���1���o��
  TW.TIOR0.BIT.IOB    = 2;   // �R���y�A�}�b�`��FTIOB���1���o��
  TW.TIOR1.BIT.IOC    = 0;   // �s���ݒ�F�o�͋֎~
  TW.TIOR1.BIT.IOD    = 0;   // �s���ݒ� : �o�͋֎~
  // ���荞�ݎ����ݒ�
  TW.GRA = 0;                // �����[�^�p:�ŏ��͒�~���Ȃ̂�0
  TW.GRB = 0;                // �E���[�^�p:�ŏ��͒�~���Ȃ̂�0
  TW.GRC = 10000;            // ���[�^�X�s�[�h�p:��~��1kHz
  TW.GRD = 2000;             // ���荞�ݎ����ݒ� : 10MHz/2000=5kHz
                             // 200us���荞��
  EI;                        // ���荞�݋���. ���W�X�^�ݒ��ɋ���.
  TW.TMRW.BIT.CTS     = 1;   // �J�E���^�X�^�[�g

  R_SW = LED_ON;             // �E�Z���TON
  L_SW = LED_ON;             // ���Z���TON
  F_SW = LED_ON;             // �O�Z���TON
}

//---------------------------------------------------------------
//  �p�����[�^�ǂݍ���
//---------------------------------------------------------------
void load_param( void )
{
  // �Z���T�������l�̌��ߑł�
  R_REF   = 0;    // ��撆���ł̉E�Z���T�l  :2018/12/21
  L_REF   = 0;    // ��撆���ł̍��Z���T�l  :2018/12/21
  // �ǂ̗L������p�������l:�e�Z���T�ǂ���ŏ��l�ƕǂȂ��l�̒��Ԓl
  R_LIM   = 0;    // �E   :2018/12/21
  L_LIM   = 0;    // ��   :2018/12/21
  F_LIM   = 0;    // �O   :2018/12/21
  // ���s�p�����[�^
  GO_STEP   = 2450; // 1��ԑO�i�X�e�b�v��
  TURN_STEP = 650;  // ����X�e�b�v��
}

//---------------------------------------------------------------
//  Timer W ���荞��(200us���ɂ��̊֐�������ɗD�悵�Ď��s�����)
//---------------------------------------------------------------
void int_timerw( void )
{
  int err_l, err_r;
  ushort acc_num, lspeed, rspeed;

  // �����[�^���荞��
  if( TW.TSRW.BIT.IMFA == 1 )
  {
    TW.TSRW.BIT.IMFA = 0;                    // ���荞�݃t���O�N���A
    TW.GRA += timerL;                        // ���̊��荞�ݎ��Ԃ��Z�b�g����
    if( speed != 0 )                         // ��~�Ȃ�p���X�������Ȃ�
    {
      TW.TCRW.BIT.TOA = 0;
      STEP++;                                // �����J�E���^�X�V
    }
    if( ldir == 0 )  L_MOT_MODE = LeftGo;    // ���]
    else             L_MOT_MODE = LeftBack;  // ���]
  }

  // �E���[�^���荞��
  if( TW.TSRW.BIT.IMFB == 1 )
  {
    TW.TSRW.BIT.IMFB = 0;                    // ���荞�݃t���O�N���A
    TW.GRB += timerR;                        // ���̊��荞�ݎ��Ԃ��Z�b�g����
    if( speed != 0 )                         // ��~�Ȃ�p���X�������Ȃ�
    {
      TW.TCRW.BIT.TOB = 0;
      STEP++;                                // �����J�E���^�X�V
    }
    if( rdir == 0 )  R_MOT_MODE = RightGo;   // ���]
    else             R_MOT_MODE = RightBack; // ���]
  }

  // ���[�^�X�s�[�h���荞��
  if( TW.TSRW.BIT.IMFC == 1 )
  {
    TW.TSRW.BIT.IMFC = 0;                    // ���荞�݃t���O�N���A
    // ���[�^�̉�������
    if( speed == 0 )                         // ���[�^��~���̏���
    {
      speed_now = 0;                         // ���x��0�ɂ���
      timerL = 10000;                        // ���荞�ݎ�����1ms�ɂ���
      timerR = 10000;
      TW.GRA = TW.TCNT + timerL;
      TW.GRB = TW.TCNT + timerR;
      TW.GRC += 10000;
    }else{
      if( speed > speed_now )       speed_now++;  // ����
      else if( speed < speed_now )  speed_now--;  // ����
      if( speed_now >= 500 ) speed_now = 499;     // �ō����x
      if( speed_now < 0     ) speed_now = 0;      // �Œᑬ�x

      acc_num = AccTable[ speed_now ];     // �����x�e�[�u������l�擾
      TW.GRC += 10000000L / (unsigned long) acc_num; // ���荞�ݎ������v�Z

      // �p������
      if( control_mode == 1 )
      {
        // �΍����v�Z
        err_l = L_SEN - L_REF;  // ���΍����v�Z
        err_r = R_SEN - R_REF;  // �E�΍����v�Z
        // �Ǐ�񂩂�΍������H
        if( L_SEN > L_LIM || R_SEN > R_LIM )
        {
          // �ǂ��炩�ɕǂ�����:�΍����傫������D�悵�ĕ␳
          if( err_l > err_r )
            err_r = -1 * err_l;
          else
            err_l = -1 * err_r;
        }else
        {
          // �����ǂȂ�:�␳�Ȃ�
          err_l = 0;
          err_r = 0;
        }
        // �΍���p���ĕ␳
        lspeed = acc_num + err_l;
        rspeed = acc_num + err_r;
      }else{  // control_mode = 0
        lspeed = acc_num ;
        rspeed = acc_num ;
      }
      timerL =  10000000L / lspeed;
      timerR =  10000000L / rspeed;
    }
  }

  if( TW.TSRW.BIT.IMFD == 1 )
  {                              // TimerW-D ���荞��
    TW.TSRW.BIT.IMFD = 0;        // ���荞�݃t���O�N���A
                                 // �Y���Ǝ��̊��荞�݂�������Ȃ�
    TW.GRD += 2000;              // ���̊��荞�݂��ēx200usec��ɐݒ�

    SENSOR_PT++;                 // �^�X�N�|�C���^�̍X�V
    if( SENSOR_PT == 5 ) SENSOR_PT = 0;  // 0-4��5�J�E���g:200us*5=1ms
                                 // �e������1ms�����Ŏ��s�����
    switch( SENSOR_PT )          // �^�X�N�|�C���^�ɏ]���ď������s��
    {
      case 0:  // 1msec�^�C�}�[&LCD�̍X�V
               wait_timer++;     // wait�֐��p�J�E���^
               LCD();            // LCD�X�V����
               break;

      case 1:  // �E�Z���T�������̑��� P11,AN0(A)
               if ( R_SW == LED_OFF ) break;         // �Z���TON/OFF�̊m�F
               AD.ADCSR.BYTE = 0x28;                 // ���ˌ��̑���J�n:AN0�P��SCAN
               while( AD.ADCSR.BIT.ADF == 0 );       // ���肪�I������܂ő҂�
               R_PRE = AD.ADDRA >> 6;       // ����f�[�^����荞�� 0 - 1023(max)
               // �E�Z���T�_�����̑��� P11,AN0(A)
               LED = LED_ON;                         // LED��_��
               LCD_wait(10);                         // ���΂炭�҂�
               AD.ADCSR.BYTE = 0x28;                 // ���ˌ��̑���J�n:AN0�P��SCAN
               while( AD.ADCSR.BIT.ADF == 0 );       // ���肪�I������܂ő҂�
               LED = LED_OFF;                        // LED������
               R_PRE = ( AD.ADDRA >> 6 ) - R_PRE;    // ����f�[�^����荞��
                                                     // ���O�v���l�Ƃ̍��������(�m�C�Y����)
               if( R_PRE <= 999 )  R_SEN = R_PRE;    // �\���������
               else                R_SEN = 0; 
               break;

      case 2:  // ���Z���T�������̑��� P11,AN1(B)
               if ( L_SW == LED_OFF ) break;         // �Z���TON/OFF�̊m�F
               AD.ADCSR.BYTE = 0x29;                 // ���ˌ��̑���J�n:AN1�P��SCAN
               while( AD.ADCSR.BIT.ADF == 0 );       // ���肪�I������܂ő҂�
               L_PRE = AD.ADDRB >> 6;                // ����f�[�^����荞�� 0 - 1023(max)
               // ���Z���T�_�����̑��� P11,AN1(B)
               LED = LED_ON;                         // LED��_��
               LCD_wait(10);                         // ���΂炭�҂�
               AD.ADCSR.BYTE = 0x29;                 // ���ˌ��̑���J�n:AN1�P��SCAN
               while( AD.ADCSR.BIT.ADF == 0 );       // ���肪�I������܂ő҂�
               LED = LED_OFF;                        // LED������
               L_PRE = ( AD.ADDRB >> 6 ) - L_PRE;    // ����f�[�^����荞��
                                                     // ���O�v���l�Ƃ̍��������(�m�C�Y����)
               if( L_PRE <= 999 )  L_SEN = L_PRE;    // �\���������
               else                L_SEN = 0;
               break;

      case 3:  // �O�Z���T�������̑��� P11,AN2(C)
               if ( F_SW == LED_OFF ) break;         // �Z���TON/OFF�̊m�F
               AD.ADCSR.BYTE = 0x2A;                 // ���ˌ��̑���J�n:AN2�P��SCAN
               while( AD.ADCSR.BIT.ADF == 0 );       // ���肪�I������܂ő҂�
               F_PRE = AD.ADDRC >> 6;        // ����f�[�^����荞�� 0 - 1023(max)
               // �O�Z���T�_�����̑��� P11,AN2(C)
               LED = LED_ON;                         // LED��_��
               LCD_wait(10);                         // ���΂炭�҂�
               AD.ADCSR.BYTE = 0x2A;                 // ���ˌ��̑���J�n:AN2�P��SCAN
               while( AD.ADCSR.BIT.ADF == 0 );       // ���肪�I������܂ő҂�
               LED = LED_OFF;                        // LED������
               F_PRE = ( AD.ADDRC >> 6 ) - F_PRE;    // ����f�[�^����荞��
                                                     // ���O�v���l�Ƃ̍��������(�m�C�Y����)
               if( F_PRE <= 999 )  F_SEN = F_PRE;
               else                F_SEN = 0;        // �\���������
               break;

      case 4:  // ���[�^�p�d���R���g���[��
               if( speed != 0 ) MotorTimer = 3000;   // ���[�^���쎞�̓^�C�}�[�Z�b�g
               else             MotorTimer--;        // ���[�^��~���̓J�E���g�_�E��
               if( MotorTimer < 0 )  MotorTimer =  0;
               // ���[�^�𓮂����Ȃ����͓d����OFF(���[�^��~����3�b��)
               if( MotorTimer == 0 )  MOTOR_EN   =  0;  // OFF
               else                   MOTOR_EN   =  1;  // ON
               break;

      default: break;
    }
  }
}

//---------------------------------------------------------------
//  wait�֐�(1ms�^�C�}�[)
//---------------------------------------------------------------
void pause( int x )
{
  wait_timer = 0;
  while( wait_timer != x ); // �I�����Ԃ܂ő҂�
}

//-------------------------------------------------------------------------
//  �L�[�I�t����
//-------------------------------------------------------------------------
void WaitKeyOff( void )
{
  // �`���^�����O�h�~����
  pause( KEY_OFF );             // �ݒ肵������([ms])�҂�
  // �S�ẴX�C�b�`��OFF�ɂȂ�܂Ń��[�v���đ҂�
  while(( SW_UP == SW_ON )||( SW_DOWN == SW_ON )||( SW_EXEC == SW_ON ));
}

//------------------------------------------------------------------------
// �r�[�v��
//------------------------------------------------------------------------
void beep(unsigned char tone,int value)
{
  TV.TCRV0.BIT.CKS  = 3;                  // �����N���b�N��/128�œ���J�n
  TV.TCORA          = beep_data[ tone ];  // �����̐ݒ�
  pause( value );                         // Beep���̒���
  TV.TCRV0.BIT.CKS  = 0;                  // Beep�����~
}

//-------------------------------------------------------------------------
//  ���[�h�\��
//-------------------------------------------------------------------------
void change_mode( int x )
{
  MODE += x;                            // ���[�h�X�V
  if( MODE >= ModeMax ) MODE = 0;       // ���[�h�������Ă���ꍇ��0�ɖ߂�
  if( MODE < 0 )  MODE = ModeMax - 1;   // ���[�h�����̏ꍇ�̓��[�h���ő�l�ɐݒ�

  if     ( MODE == 0 ) mode0( DISP );   // Mode0:
  else if( MODE == 1 ) mode1( DISP );   // Mode1:
  else if( MODE == 2 ) mode2( DISP );   // Mode2:
  else if( MODE == 3 ) mode3( DISP );   // Mode3:
  else if( MODE == 4 ) mode4( DISP );   // Mode4:
  else if( MODE == 5 ) mode5( DISP );   // Mode5:
  else if( MODE == 6 ) mode6( DISP );   // Mode6:

  else if( MODE == 7 ) mode7( DISP );   // Mode7:

}

//-------------------------------------------------------------------------
//  ���[�h����
//-------------------------------------------------------------------------
void exec_mode( void )
{
  if     ( MODE == 0 ) mode0( EXEC );   // Mode0:
  else if( MODE == 1 ) mode1( EXEC );   // Mode1:
  else if( MODE == 2 ) mode2( EXEC );   // Mode2:
  else if( MODE == 3 ) mode3( EXEC );   // Mode3:
  else if( MODE == 4 ) mode4( EXEC );   // Mode4:
  else if( MODE == 5 ) mode5( EXEC );   // Mode5:
  else if( MODE == 6 ) mode6( EXEC );   // Mode6:

  else if( MODE == 7 ) mode7( EXEC );   // Mode7:

}

//-------------------------------------------------------------------------
//  Mode0 : �Z���T�`�F�b�N
//-------------------------------------------------------------------------
void mode0( int x )
{
  if( x == DISP )  // DISP���[�h�̏ꍇ
  {
    // ���[�h���e�\��
    LCD_print( 0, "0:Sensor" );
    LCD_print( 8, "        " );
    pause( 1000 );
    LCD_print( 0, "  F     " );
    LCD_print( 8, "L   R   " );
    return;                     // �ȉ��̎��s���������Ȃ��Ŗ߂�
  }

  // ���s���[�h�̏ꍇ
  L_REF = 0; R_REF = 0;         // �f�[�^������

  for( x = 0; x < 32; x++ )     // �f�[�^����(32�|�C���g)
  {
    L_REF += L_SEN;
    R_REF += R_SEN;
    pause(1);
  }

  R_REF = R_REF / 32;           // ����f�[�^�𕽋ω�
  L_REF = L_REF / 32;

  LCD_print( 0, " L    R " );
  LCD_print( 8, "        " );
  LCD_dec_out(  8, L_REF, 3 );  // ���Z���T�l��LCD�ɕ\��
  LCD_dec_out( 13, R_REF, 3 );  // �E�Z���T�l��LCD�ɕ\��

  pause( 2000 );                // 2�b�ԕ\��
}

//-------------------------------------------------------------------------
//  Mode1 : ���[�^�e�X�g
//-------------------------------------------------------------------------
void mode1(int x)
{
  if( x == DISP )  // DISP���[�h�̏ꍇ
  {
    // ���[�h���e�\��
    LCD_print( 0, "1:M-TEST" );
    LCD_print( 8, "        " );
    return;                     // �ȉ��̎��s���������Ȃ��Ŗ߂�
  }

  // ���s���[�h�̏ꍇ
  LCD_print( 8,"SPD=     ");
  rdir = 0; ldir = 0;           // ��]�����𒼐i
  control_mode = 1;             // �������s�p�p�����䂠��

  while(1){
    LCD_dec_out( 12, speed, 4 );
    if( SW_UP   == 0 ) { speed += 100; WaitKeyOff(); }
    if( SW_DOWN == 0 ) { speed -= 100; WaitKeyOff(); }
    if( SW_EXEC == 0 ) { speed = 0; return; }

    if( speed > 500 )    speed = 500;
    else if( speed < 0 )  speed = 0;
  }
}

//-------------------------------------------------------------------------
//  Mode2 : 1��ԑO�i
//-------------------------------------------------------------------------
void mode2(int x)
{
  if( x == DISP )  // DISP���[�h�̏ꍇ
  {
    // ���[�h���e�\��
    LCD_print( 0, "2: 1 GO " );
    LCD_print( 8, "STEP    " );
    return;                     // �ȉ��̎��s���������Ȃ��Ŗ߂�
  }

  // ���s���[�h�̏ꍇ
  while(1){
    LCD_dec_out( 12, GO_STEP, 4 );
    if( SW_UP   == 0 ) { GO_STEP += 10; WaitKeyOff(); }
    if( SW_DOWN == 0 ) { GO_STEP -= 10; WaitKeyOff(); }
    if( SW_EXEC == 0 ) { WaitKeyOff();  com_go( 1 );  com_stop(); }
  }
}

//-------------------------------------------------------------------------
//  Mode3 : N��ԑO�i
//-------------------------------------------------------------------------
void mode3(int x)
{
  int n = 5;                    // �O�i��Ԑ��F�����l 5
  if( x == DISP )  // DISP���[�h�̏ꍇ
  {
    // ���[�h���e�\��
    LCD_print( 0, "3: N GO " );
    LCD_print( 8, "     N  " );
    return;                     // �ȉ��̎��s���������Ȃ��Ŗ߂�
  }

  // ���s���[�h�̏ꍇ
  LCD_dec_out( 8, GO_STEP, 4 );
  while(1){
    LCD_dec_out( 14, n, 2 );
    if( SW_UP   == 0 ) { n++; WaitKeyOff(); }
    if( SW_DOWN == 0 ) { n--; WaitKeyOff(); }
    if( SW_EXEC == 0 ) { WaitKeyOff();  com_go( n );  com_stop(); }
  }
}

//-------------------------------------------------------------------------
//  Mode4 : 180�^�[��R
//-------------------------------------------------------------------------
void mode4( int x )
{
  if( x == DISP )  // DISP���[�h�̏ꍇ
  {
    // ���[�h���e�\��
    LCD_print( 0, "4: TURN " );
    LCD_print( 8, "        " );
    return;                     // �ȉ��̎��s���������Ȃ��Ŗ߂�
  }

  // ���s���[�h�̏ꍇ
  while(1){
    LCD_dec_out( 10, TURN_STEP, 4 );
    if( SW_UP   == 0 ) { TURN_STEP += 10; WaitKeyOff(); }
    if( SW_DOWN == 0 ) { TURN_STEP -= 10; WaitKeyOff(); }
    if( SW_EXEC == 0 ) { com_turn(2); com_stop(); }
  }
}

//-------------------------------------------------------------------------
//  Mode5 : �T�����s
//-------------------------------------------------------------------------
void mode5( int x )
{
  if( x == DISP )  // DISP���[�h�̏ꍇ
  {
    // ���[�h���e�\��
    LCD_print( 0, "5:Search" );
    LCD_print( 8, "Spd  200" );
    return;                     // �ȉ��̎��s���������Ȃ��Ŗ߂�
  }

  // ���s���[�h�̏ꍇ
  pos_x = 0; pos_y = 0; head = 0;
  // �����T��
  mouse_search( GOAL_X, GOAL_Y, 200, S_MODE );  // �s���̒T��
  mouse_search( 0, 0, 200, S_MODE );            // �A��̒T��
}

//-------------------------------------------------------------------------
//  Mode6 : �񎟑��s
//-------------------------------------------------------------------------
void mode6( int x )
{
  if( x == DISP )  // DISP���[�h�̏ꍇ
  {
    // ���[�h���e�\��
    LCD_print( 0, "6:Try   " );
    LCD_print( 8, "Spd  200" );
    return;                     // �ȉ��̎��s���������Ȃ��Ŗ߂�
  }

  // ���s���[�h�̏ꍇ
  // �񎟑��s
  pos_x = 0; pos_y = 0; head = 0;
  mouse_search( GOAL_X, GOAL_Y, 200, T_MODE );
}

//-------------------------------------------------------------------------
//  Mode7 : myself
//-------------------------------------------------------------------------
void mode7( int x )
{
  if( x == DISP )  // DISP���[�h�̏ꍇ
  {
    // ���[�h���e�\��
    LCD_print( 0, "7:myself" );
    LCD_print( 8, "--------" );
    return;                     // �ȉ��̎��s���������Ȃ��Ŗ߂�
  }

  // ���s���[�h�̏ꍇ
  com_back(1);
}

//-------------------------------------------------------------------------
//  �T���֐�
//-------------------------------------------------------------------------
void mouse_search( int goal_x, int goal_y, int spd, int mode )
{
  short motion;
  countdown();                  // �J�E���g�_�E��

  while( 1 ){
    // �P�̃��[�v�͋�Ԓ��S���玟�̋�Ԓ��S�܂�
    // �ŏ��ɔ���撼�i
    control_mode = 1;             // �p������ON
    rdir = 0; ldir = 0;           // ��]�����𒼐i
    STEP = 0;                     // �����J�E���^���Z�b�g
    speed = spd;                  // ���x�ݒ�

    // ���W�X�V
    if     ( head == 0 ) pos_y++; // �k���� y+1
    else if( head == 1 ) pos_x++; // ������ x+1
    else if( head == 2 ) pos_y--; // ����� y-1
    else if( head == 3 ) pos_x--; // ������ x-1
    
    // �|�e���V����MAP�v�Z
    make_potential( goal_x, goal_y, mode );
    
    while( STEP < GO_STEP / 2 );  // ����Ԑi��

    // ���܂Ői�񂾂�
    // �Ǐ��擾��MAP�f�[�^�㏑���i�T�����s���̂݁j
    if( mode == S_MODE )
      make_map_data(); 

    // �����@�ōs������
    motion = search_adachi();
    
    // �S�[�����̗�O�����i��Ō��߂��s�����㏑�������j
    if( pos_x == goal_x && pos_y == goal_y )
      motion = 4;                       // �S�[�����B�F���]��~

    // �s�������s
    switch( motion ){
      // ���i
      case  0 : while( STEP < GO_STEP );  // �c�蔼��Ԑi��
                head_change = 0;          // �i�s�����X�V�ϐ���O�ɐݒ�
                break;
      // �E��
      case  1 : while( STEP < GO_STEP - speed_now * 2 );  // ��������c���Ē��i
                speed = 1;
                while( STEP < GO_STEP );  // �c��X�e�b�v���Ō���
                com_turn( 0 );            // �E90�x����
                head_change = 1;          // �i�s�����X�V�ϐ����E�ɐݒ�
                break;
      // ���]
      case  2 : while( STEP < GO_STEP - speed_now * 2 );  // ��������c���Ē��i
                speed = 1;
                while( STEP < GO_STEP );  // �c��X�e�b�v���Ō���
                com_turn( 2 );            // ���]
                head_change = 2;          // �i�s�����X�V�ϐ�����ɐݒ�
                break;
      // ����
      case  3 : while( STEP < GO_STEP - speed_now * 2 );  // ��������c���Ē��i
                speed = 1;
                while( STEP < GO_STEP );  // �c��X�e�b�v���Ō���
                com_turn( 1 );            // ��90�x����
                head_change = 3;          // �i�s�����X�V�ϐ������ɐݒ�
                break;
      // ���]��~
      case  4 : while( STEP < GO_STEP - speed_now * 2 );  // ��������c���Ē��i
                speed = 1;
                while( STEP < GO_STEP );  // �c��X�e�b�v���Ō���
                com_turn( 2 );            // ���]
                com_stop();               // ��~
                head_change = 2;          // �i�s�����X�V�ϐ�����ɐݒ�
                head = ( head + head_change ) & 0x03; // �ڍׂ͉����Q��
                finish();                 // �S�[����
                return;                   // ���[�v�I��
                break;
      // ���̑�
      default : com_stop();               // ��~
                head_change = 0;          // �i�s�����X�V�ϐ���O�ɐݒ�
                head = ( head + head_change ) & 0x03; // �ڍׂ͉����Q��
                return;                   // ���[�v�I��
                break;
    }
    
    // �i�s�����X�V�ϐ�head_change��p���Č��݂̐i�s����head���X�V
    head = ( head + head_change ) & 0x03; // �X�V���l�����Z����2�i����2���Ń}�X�N
                                          // 00 -> 01 -> 10 -> 11 -(�}�X�N)-> 00
  }
}

//-------------------------------------------------------------------------
//  ���i���W���[�� (N��ԑO�i)
//-------------------------------------------------------------------------
void com_go( int n )
{
  control_mode = 1;                       // �������s�p�p������
  STEP = 0;                               // �����J�E���^�N���A
  rdir = 0; ldir = 0;                     // ��]�����𒼐i

  // �������[�h
  speed = 200;        // �ڕW���x�ݒ�
  while( speed > speed_now );                   // �ڕW���x�ɂȂ�܂ŉ���
  // �葬���[�h
  speed = speed_now;  // ������̑��x
  while( STEP < GO_STEP * n - speed_now * 2 );  // �����X�e�b�v�����c���Ē葬�ړ�
                                                // �S�̃X�e�b�v��-�����p�X�e�b�v��
  // �������[�h
  speed = 1;          // �Œᑬ�x�ݒ�
  while( STEP < GO_STEP * n );                  // �c��̃X�e�b�v���Ō���
}

//-------------------------------------------------------------------------
//  ��ރ��W���[�� (N��Ԍ��)
//-------------------------------------------------------------------------
void com_go( int n )
{
  control_mode = 1;                       // �������s�p�p������
  STEP = 0;                               // �����J�E���^�N���A
  rdir = 1; ldir = 1;                     // ��]�����𒼐i

  // �������[�h
  speed = 200;        // �ڕW���x�ݒ�
  while( speed > speed_now );                   // �ڕW���x�ɂȂ�܂ŉ���
  // �葬���[�h
  speed = speed_now;  // ������̑��x
  while( STEP < GO_STEP * n - speed_now * 2 );  // �����X�e�b�v�����c���Ē葬�ړ�
                                                // �S�̃X�e�b�v��-�����p�X�e�b�v��
  // �������[�h
  speed = 1;          // �Œᑬ�x�ݒ�
  while( STEP < GO_STEP * n );                  // �c��̃X�e�b�v���Ō���
}

//-------------------------------------------------------------------------
//  ��~���W���[��
//-------------------------------------------------------------------------
void com_stop( void )
{
  control_mode = 0;           // �p�����䖳��
  rdir = 0; ldir = 0;         // ���[�^�̉�]������O�i
  STEP = 0;                   // �����J�E���^�����Z�b�g
  speed = 0;  speed_now = 0;  // ���[�^�̐���p�̕ϐ������Z�b�g
  pause(100);                 // 0.1�b���[�^���~
}

//-------------------------------------------------------------------------
//  ���񃂃W���[�� (0:R90 1:L90 2:R180 3:L180)
//-------------------------------------------------------------------------
void com_turn( int t_mode )
{
  short T_STEP;

  com_stop();                                             // ��~
  control_mode = 0;                                       // �p������Ȃ�
  if     ( t_mode == 0 ) { T_STEP = TURN_STEP; rdir = 1; ldir = 0; } // �E�X�O�x
  else if( t_mode == 1 ) { T_STEP = TURN_STEP; rdir = 0; ldir = 1; } // ���X�O�x
  else if( t_mode == 2 ) { T_STEP = TURN_STEP * 2; rdir = 1; ldir = 0; } // �E���]
  else if( t_mode == 3 ) { T_STEP = TURN_STEP * 2; rdir = 0; ldir = 1; } // �����]

  // �������[�h
  speed = 100;        // �ڕW���x�ݒ�
  while( speed > speed_now );                   // �ڕW���x�ɂȂ�܂ŉ���
  // �葬���[�h
  speed = speed_now;  // ������̑��x
  while( STEP < T_STEP - speed_now * 2 );       // �����X�e�b�v�����c���Ē葬�ړ�
                                                // �S�̃X�e�b�v��-�����p�X�e�b�v��
  // �������[�h
  speed = 1;          // �Œᑬ�x�ݒ�
  while( STEP < T_STEP );                       // �c��̃X�e�b�v���Ō���
}

//-------------------------------------------------------------------------
//  �J�E���g�_�E��
//-------------------------------------------------------------------------
void countdown( void )
{
  beep( 0, 850 );
  beep( 1, 150 );
  beep( 0, 850 );
  
  R_SW = LED_OFF;        // �E�Z���TOFF
  L_SW = LED_OFF;        // ���Z���TOFF
  F_SW = LED_OFF;        // �O�Z���TOFF
  beep( 1, 150 );
  beep( 0, 850 );

  R_SW = LED_ON;         // �E�Z���TON
  L_SW = LED_ON;         // ���Z���TON
  F_SW = LED_ON;         // �O�Z���TON
  beep( 13, 1000 );
}

//-------------------------------------------------------------------------
//  �S�[����
//-------------------------------------------------------------------------
void finish( void )
{
  beep( 8 , 150 );
  beep( 0 , 150 );
  beep( 8 , 150 );
  beep( 0 , 50 );
  beep( 13 , 500);
}

//-------------------------------------------------------------------------
//  �ǂ̃Z���V���O
//-------------------------------------------------------------------------
int get_wall_data( void )
{
  short wall;

  // �Z���T�f�[�^����́C臒l�Ɣ�r���ĕǂ̗L���𔻒�
  wall = 0;
  if( F_SEN > F_LIM )  wall |= 0x01; // �O�ǂ���
  if( R_SEN > R_LIM )  wall |= 0x02; // �E�ǂ���
  if( L_SEN > L_LIM )  wall |= 0x08; // ���ǂ���
  // ��ǂ͂���킯�Ȃ��̂Ō��Ȃ�

  return( wall );
}

//-------------------------------------------------------------------------
// MAP�f�[�^�̌���
// �T���L�^ bit 7 6 5 4 = �� �� �� �k / �l = 1:���T�� 0:���T��
// �Ǐ��   bit 3 2 1 0 = �� �� �� �k / �l = 1:�ǗL�� 0:�ǖ���
//-------------------------------------------------------------------------

//-------------------------------------------------------------------------
//  MAP�f�[�^������
//-------------------------------------------------------------------------
void clear_map( void )
{
  int x, y;
  // �S�Ă̋�Ԃ�ǂȂ������T���ɏ�����
  for( y = 0 ; y < 16 ; y++ )
    for( x = 0 ; x < 16 ; x++ )
      map[ x ][ y ] = 0x00;

  // �����̊O�ǁix=0�Cy=0�`15�j���㏑��
  for( y = 0 ; y < 16 ; y++ )
    map[ 0 ][ y ] = 0x88;  // ���̂݊��T��(8)�����̂ݕǂ���(8)
  // �쑤�̊O�ǁix=0�`15�Cy=0�j���㏑��
  for( x = 0 ; x < 16 ; x++ )
    map[ x ][ 0 ] = 0x44;  // ��̂݊��T��(4)����̂ݕǂ���(4)
  // �����̊O�ǁix=15�Cy=0�`15�j���㏑��
  for( y = 0 ; y < 16 ; y++ )
    map[ 15 ][ y ] = 0x22; // ���̂݊��T��(2)�����̂ݕǂ���(2)
  // �k���̊O�ǁix=0�`15�Cy=15�j���㏑��
  for( x = 0 ; x < 16 ; x++ )
    map[ x ][ 15 ] = 0x11; // �k�̂݊��T��(1)���k�̂ݕǂ���(1)

  // �X�^�[�g��ԁix=0�Cy=0�j�̏㏑��
  map[ 0 ][ 0 ] = 0xfe;  // ���쓌�k���T��(8+4+2+1=f)�����쓌�ǂ���(8+4+2=e)
  // �X�^�[�g���1�E�ix=1�Cy=0�j�̏㏑��
  map[ 1 ][ 0 ] = 0xcc;  // ������T��(8+4=c)������ǂ���(8+4=c)

  // ����p�ix=0�Cy=15�j�̏㏑��
  map[ 0 ][ 15 ] = 0x99; // ���k���T��(8+1=9)�����k�ǂ���(8+1=9)
  // �E���p�ix=15�Cy=0�j�̏㏑��
  map[ 15 ][ 0 ] = 0x66; // �쓌���T��(4+2=6)���쓌�ǂ���(4+2=6)
  // �E��p�ix=15�Cy=15�j�̏㏑��
  map[ 15 ][ 15 ] = 0x33;// ���k���T��(2+1=3)�����k�ǂ���(2+1=3)
}

//-------------------------------------------------------------------------
//  �Z���T��񂩂�T���L�^���Ǐ���MAP�f�[�^�ɍX�V
//-------------------------------------------------------------------------
void make_map_data( void )
{
  uchar wall;

  // �Ǐ��擾
  wall = get_wall_data();
  // �������킹�����̂��߂ɏ��4bit�ɉ���4bit�̕Ǐ����R�s�[
  wall = ( wall & 0x0f ) | ( wall << 4 );
  // �}�E�X�̐i�s�����ɂ��킹�ĕǃf�[�^�����H
  if     ( head == 1 ) wall = wall >> 3; // �k���O�̏��𓌂��O�ɉ��H
  else if( head == 2 ) wall = wall >> 2; // �k���O�̏���삪�O�ɉ��H
  else if( head == 3 ) wall = wall >> 1; // �k���O�̏��𐼂��O�ɉ��H
  // ���쓌�k��T���ς݂ɂ���
  wall |= 0xf0;
  // �Ǐ���MAP�f�[�^�ɏ㏑��
  map[ pos_x ][ pos_y ] = wall;

  // ���쓌�k�̗׋���MAP�f�[�^���㏑��

  // ���݋��̓��Ǐ���1�E���̐��Ǐ��Ƃ��ď㏑�����鏈��
  // ���������ڍׂɐ����D�c��3�i���C���C��j�͂܂Ƃ߂ċL�q
  if( pos_x != 15 ){  // ��ԓ����̋��̎��ȊO
    // �E���̐������i�T���L�^���Ǐ��j������
    map[ pos_x + 1 ][ pos_y ] &= 0x77;
    // �E���̐����T���L�^�����T���Ƃ���
    map[ pos_x + 1 ][ pos_y ] |= 0x80;
    // ���݋��̓������𐼑����ɕϊ����ĉE���̐����Ǐ��ɏ㏑��
    map[ pos_x + 1 ][ pos_y ] |= ( map[ pos_x ][ pos_y ] << 2 ) & 0x08;
  }
  
  // ���݋��̓�Ǐ���1�����̖k�Ǐ��Ƃ��ď㏑�����鏈��
  if(pos_y!=0) map[pos_x][pos_y-1]=(map[pos_x][pos_y-1]&0xee)|0x10|((wall>>2)&0x01);
  // ���݋��̐��Ǐ���1�����̓��Ǐ��Ƃ��ď㏑�����鏈��
  if(pos_x!=0) map[pos_x-1][pos_y]=(map[pos_x-1][pos_y]&0xdd)|0x20|((wall>>2)&0x02);
  // ���݋��̖k�Ǐ���1����̓�Ǐ��Ƃ��ď㏑�����鏈��
  if(pos_y!=15)map[pos_x][pos_y+1]=(map[pos_x][pos_y+1]&0xbb)|0x40|((wall<<2)&0x04);
}

//-------------------------------------------------------------------------
//  �������i�|�e���V������j�쐬
//-------------------------------------------------------------------------
void make_potential( int gx, int gy, int mode )
{
  uchar check_num, flg;
  uchar x,y;

  // �|�e���V����MAP������(�S�čő�l255�ɂ���)
  for( y = 0 ; y < 16 ; y++ )
    for( x = 0 ; x < 16 ; x++ )
      p_map[ x ][ y ] = 255;

  // �S�[�����W�Ƀ|�e���V����0����������
  p_map[ gx ][ gy ] = 0;

  check_num = 0;
  do{
    flg = 0;  // �ύX�t���O������
    for( y = 0 ; y < 16 ; y++ ){
      for( x = 0 ; x < 16 ; x++ ){
        if( p_map[ x ][ y ] == check_num ){  // ����Ώۋ��Ƃ���|�e���V����
          if( mode == S_MODE ){

            // �T�����s(Search Mode)
            // �k���̕ǂ��Ȃ��ꍇ�F�k���̃|�e���V������Ώۋ��̃|�e���V�������+1
            if((( map[ x ][ y ] & 0x01 ) == 0 ) && ( y != 15 )){
              if( p_map[ x ][ y + 1 ] == 255 ){// �܂��|�e���V�����������ĂȂ����
                p_map[ x ][ y + 1 ] = check_num + 1;
                flg = 1;  // �ύX�����̂Ńt���OON
              }
            }
            // �����̕ǂ����l�ɏ���
            if((( map[ x ][ y ] & 0x02 ) == 0 ) && ( x != 15 ))
              if(p_map[x+1][y]==255){p_map[x+1][y]=check_num+1;flg=1;}
            // �쑤�̕ǂ����l�ɏ���
            if((( map[ x ][ y ] & 0x04 ) == 0 ) && ( y != 0 ))
              if(p_map[x][y-1]==255){p_map[x][y-1]=check_num+1;flg=1;}
            // �����̕ǂ����l�ɏ���
            if((( map[ x ][ y ] & 0x08 ) == 0 ) && ( x != 0 ))
              if(p_map[x-1][y]==255){p_map[x-1][y]=check_num+1;flg=1;}

          }else{

           // �񎟑��s(Try Mode)
           // �k�����ǂȂ������T���̏ꍇ(�ǂȂ��ł����T���̓|�e���V����255�̂܂�)
            // �k���̃|�e���V������Ώۋ��̃|�e���V�������+1
            if((( map[ x ][ y ] & 0x11 ) == 0x10 ) && ( y != 15 )){
              if( p_map[ x ][ y + 1 ] == 255 ){// �܂��|�e���V�����������ĂȂ����
                p_map[ x ][ y + 1 ] = check_num + 1;
                flg = 1;  // �ύX�����̂Ńt���OON
              }
            }
            // �����̕ǂ����l�ɏ���
            if((( map[ x ][ y ] & 0x22 ) == 0x20 ) && ( x != 15 ))
              if(p_map[x+1][y]==255){p_map[x+1][y]=check_num+1;flg=1;}
            // �쑤�̕ǂ����l�ɏ���
            if((( map[ x ][ y ] & 0x44 ) == 0x40 ) && ( y != 0 ))
              if(p_map[x][y-1]==255){p_map[x][y-1]=check_num+1;flg=1;}
            // �����̕ǂ����l�ɏ���
            if((( map[ x ][ y ] & 0x88 ) == 0x80 ) && ( x != 0 ))
              if(p_map[x-1][y]==255){p_map[x-1][y]=check_num+1;flg=1;}

          }
        }
      }
    }
    check_num++;      // ���̃��[�v�̂��߂ɑΏۃ|�e���V������+1
  }while( flg != 0 ); // ����̃��[�v�ŕύX�ӏ���������΍쐬����
}

//-------------------------------------------------------------------------
//  �T���F����@
//-------------------------------------------------------------------------
int search_left_hand( void ){
  short wall_data, motion;

  wall_data = get_wall_data();  // �Ǐ��擾

  // �Ǐ���p���Ď��̍s��������i����@�j
    // motion = 0:���i / 1:�E�� / 2:���] / 3:���� / 4:���]��~
    switch( wall_data ){
      case  0x00  : motion = 3; break;  // ���ɕǂȂ�:����
      case  0x01  : motion = 3; break;  // ���ɕǂȂ�:����
      case  0x02  : motion = 3; break;  // ���ɕǂȂ�:����
      case  0x03  : motion = 3; break;  // ���ɕǂȂ�:����
      case  0x04  : motion = 3; break;  // ���ɕǂȂ�:����
      case  0x05  : motion = 3; break;  // ���ɕǂȂ�:����
      case  0x06  : motion = 3; break;  // ���ɕǂȂ�:����
      case  0x07  : motion = 3; break;  // ���ɕǂȂ�:����
      case  0x08  : motion = 0; break;  // ���ɕ�,�O�ɕǂȂ�:���i
      case  0x09  : motion = 1; break;  // ���ɕ�,�O�ɕ�,�E�ɕǂȂ�:�E��
      case  0x0a  : motion = 0; break;  // ���ɕ�,�O�ɕǂȂ�:���i
      case  0x0b  : motion = 2; break;  // ���ɕ�,�O�ɕ�,�E�ɕ�:���]
      default     : motion = 4; break;  // ���ɕ�:���蓾�Ȃ��̂Œ�~
  }
  return( motion );
}

//-------------------------------------------------------------------------
//  �T���F�g������@
//-------------------------------------------------------------------------
int search_ex_left_hand( void ){
  short wall_data, motion, val, min_val;

  wall_data = map[ pos_x ][ pos_y ];  // ���݋��̕Ǐ��擾

  min_val = 8;  // �v�Z�����D��x�̍ő�l+1�������l�ɐݒ�

  // �k�����̗D��x�̌v�Z
  if(( wall_data & 0x01 ) == 0 ){     // �k�����ɕǂ������Ƃ�
    // 1.�����ɂ��D��x�̌v�Z
    // �}�E�X���猩�����̕ǂ̕����ƗD��x ��:0, �O:1�C�E:2�C��:3
    // head=0�i�}�E�X�̓����k�j�̏ꍇ�͗D��x1�i�k�͑O�j
    // head=1�i�}�E�X�̓������j�̏ꍇ�͗D��x0�i�k�͍��j
    // head=2�i�}�E�X�̓�����j�̏ꍇ�͗D��x3�i�k�͌�j
    // head=3�i�}�E�X�̓������j�̏ꍇ�͗D��x2�i�k�͉E�j
    val = (( 3 - head ) + 2 ) & 0x03;

    // 2.���T���^���T���ɂ��D��x�̌v�Z
    // ���T��:0�C���T��:+4
    if(( map[ pos_x ][ pos_y + 1 ] & 0xf0 ) == 0xf0 ) val += 4;

    // 3.���̕ǂ��D�悩�ǂ����𔻒f
    // �����܂ł̌v�Z�ňȉ��̗D��x�̂ǂꂩ�ɂȂ�
    // 0:�k�������}�E�X�̍����Ŗ��T��
    // 1:�k�������}�E�X�̑O���Ŗ��T��
    // 2:�k�������}�E�X�̉E���Ŗ��T��
    // 3:�k�������}�E�X�̌㑤�Ŗ��T���i���L�蓾�Ȃ��j
    // 4:�k�������}�E�X�̍����Ŋ��T��
    // 5:�k�������}�E�X�̑O���Ŋ��T��
    // 6:�k�������}�E�X�̉E���Ŋ��T��
    // 7:�k�������}�E�X�̌㑤�Ŋ��T��

    if( val < min_val ){
      min_val = val;  // �ŏ��l�̍X�V
      motion = 0;     // �ړ����ׂ�������k�ɐݒ�
    }
  }

  // �������̗D��x�̌v�Z
  if(( wall_data & 0x02 ) == 0 ){     // �������ɕǂ������Ƃ�
    val = (( 3 - head ) + 3 ) & 0x03;
    if(( map[ pos_x + 1 ][ pos_y ] & 0xf0 ) == 0xf0 ) val += 4;
    if( val < min_val ){
      min_val = val;  // �ŏ��l�̍X�V
      motion = 1;     // �ړ����ׂ������𓌂ɐݒ�
    }
  }

  // ������̗D��x�̌v�Z
  if(( wall_data & 0x04 ) == 0 ){     // ������ɕǂ������Ƃ�
    val = (( 3 - head ) + 0 ) & 0x03;
    if(( map[ pos_x ][ pos_y - 1 ] & 0xf0 ) == 0xf0 ) val += 4;
    if( val < min_val ){
      min_val = val;  // �ŏ��l�̍X�V
      motion = 2;     // �ړ����ׂ��������ɐݒ�
    }
  }

  // �������̗D��x�̌v�Z
  if(( wall_data & 0x08 ) == 0 ){     // �������ɕǂ������Ƃ�
    val = (( 3 - head ) + 1 ) & 0x03;
    if(( map[ pos_x - 1 ][ pos_y ] & 0xf0 ) == 0xf0 ) val += 4;
    if( val < min_val ){
      min_val = val;  // �ŏ��l�̍X�V
      motion = 3;     // �ړ����ׂ������𐼂ɐݒ�
    }
  }

  // �ړ����ׂ���������s��������
  // (�ڕW����-���ݕ���)��2�i����2���Ń}�X�N
  // �����Z�̌��ʂ����̏ꍇ��2�̕␔�Ń}�X�N�����
  // 11 -> 10 -> 01 -> 00 -> (-1)=11 -> (-2)=10 ...
  motion = ( motion - head ) & 0x03;

  return( motion );
}

//-------------------------------------------------------------------------
//  �T���F�����@
//-------------------------------------------------------------------------
int search_adachi( void )
{
  uchar wall_data, motion;
  short val, min_val;

  // ���݋��̕Ǐ��擾
  wall_data = map[ pos_x ][ pos_y ];

  // �v�Z�����D��x�̍ő�l�������l�ɐݒ�
  min_val = 1025;  // ���|�e���V�����ő�l+1 255*4+4 +1 =1025

  // ���͂S�̕����ɑ΂��ėD��x���v�Z���C
  // ��ԗD��x�������i�l���������j���Ɉړ�����D
  // �D��x�̓|�e���V�����C���^���T���C���i�����̏��D
  // ��F�|�e���V������0�̏ꍇ����{�D��x��0*4+4=4
  // �����T���Ȃ�-2�C���i�Ȃ�-1�̌��Z����
  // 4:���T�������i�ȊO
  // 3:���T�������i
  // 2:���T�������i�ȊO
  // 1:���T�������i
  // �D��x���������ʂ̏ꍇ�͖k���쐼�̏��ɗD�悳���

  // �k�����̗D��x�̌v�Z
  if(( wall_data & 0x01 ) == 0 ){     // �k�����ɕǂ������Ƃ�
    // 1.�|�e���V���������Ɋ�{�D��x���v�Z
    val = p_map[ pos_x ][ pos_y + 1 ] * 4 + 4;
    // 2.�����ɂ��D��x�̌v�Z
    // �k�������i�s�����������ꍇ�F-1(�D��x��1�グ��)
    if( head == 0 )  val -= 1;
    // 3.���T���^���T���ɂ��D��x�̌v�Z
    // ���T��:-2(�D��x��2�グ��)�C���T��:0
    if(( map[ pos_x ][ pos_y + 1 ] & 0xf0 ) != 0xf0 )  val -= 2;
    // �ŏ��l�̍X�V
    if( val < min_val ){
      min_val = val;
      motion = 0;  // �ړ����ׂ�������k�ɐݒ�
    }
  }

  // �������̗D��x�̌v�Z
  if(( wall_data & 0x02 ) == 0 ){     // �������ɕǂ������Ƃ�
    val = p_map[ pos_x + 1 ][ pos_y ] * 4 + 4;
    if( head == 1 )  val -= 1;
    if(( map[ pos_x + 1 ][ pos_y ] & 0xf0 ) != 0xf0 )  val -= 2;
    if( val < min_val ){
      min_val = val;
      motion = 1;  // �ړ����ׂ������𓌂ɐݒ�
    }
  }

  // ������̗D��x�̌v�Z
  if(( wall_data & 0x04 ) == 0 ){     // ������ɕǂ������Ƃ�
    val = p_map[ pos_x ][ pos_y - 1 ] * 4 + 4;
    if( head == 2 )  val -= 1;
    if(( map[ pos_x ][ pos_y - 1 ] & 0xf0 ) != 0xf0 )  val -= 2;
    if( val < min_val ){
      min_val = val;
      motion = 2;  // �ړ����ׂ��������ɐݒ�
    }
  }

  // �������̗D��x�̌v�Z
  if(( wall_data & 0x08 ) == 0 ){     // �������ɕǂ������Ƃ�
    val = p_map[ pos_x - 1 ][ pos_y ] * 4 + 4;
    if( head == 3 )  val -= 1;
    if(( map[ pos_x - 1 ][ pos_y ] & 0xf0 ) != 0xf0 )  val -= 2;
    if( val < min_val ){
      min_val = val;
      motion = 3;  // �ړ����ׂ������𐼂ɐݒ�
    }
  }

  // �ړ����ׂ���������s��������
  motion = ( motion - head ) & 0x03;

  return( motion );
}
