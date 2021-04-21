//***************************************************************
//  ユニバーサルキット 3.0.1 メインプログラム
//                                          2016/09/26 H. Suzuki
//   修正 2018/7/4 5 12/21 ito 学生引き渡し用に修正
//
//  2016/09/16  Sample. 01 ：書き込みチェック用
//  2016/09/16  Sample. 02 ：LED点滅
//  2016/09/16  Sample. 03 ：LCD表示
//  2016/09/16  Sample. 04 ：Timer W - Dのみ
//  2016/09/16  Sample. 05 ：SW 追加
//  2016/09/16  Sample. 06 ：Mode System 追加
//  2016/09/16  Sample. 07 ：A/D 追加
//  2016/09/16  Sample. 08 ：Beep 追加
//  2016/09/26  Sample. 09 ：Sensor 追加
//  2016/09/26  Sample. 10 ：Motor1(手動駆動) 追加
//  2016/09/26  Sample. 11 ：Motor2(割り込み駆動) 追加
//  2016/09/26  Sample. 12 ：Motor3(姿勢制御), MotorTest 追加
//  2016/09/26  Sample. 13 ：STEP, 1&N区間前進 追加
//  2016/09/26  Sample. 14 ：TURN 追加
//  2016/09/26  Sample. 15 ：迷路周回(探索関数) 追加
//  2016/09/26  Sample. 16 ：左手法 追加
//  2016/09/26  Sample. 17 ：座標更新 追加
//  2016/09/26  Sample. 18 ：マッピング 追加，探索法関数化
//  2016/09/26  Sample. 19 ：拡張左手法 追加
//  2016/09/26  Sample. 20 ：足立法 追加
//  2016/09/26  Sample. 21 ：二次走行 追加
//
//***************************************************************

#include "3694.h"
#include "140225LCD.h"        // LCD用関数ヘッダ
#include "160926AccTable.h"   // モータ速度用ヘッダ

//---------------------------------------------------------------
//  型宣言
//---------------------------------------------------------------
#define  uchar    unsigned char
#define  ushort   unsigned short
#define  vshort   volatile short
#define  vushort  volatile unsigned short

//-------------------------------------------------------------------------
//  ポート定義
//-------------------------------------------------------------------------
#define   CPU_LED     IO.PDR2.BIT.B0    // CPU層LED
#define   LED         IO.PDR1.BIT.B0    // LEDスイッチ
#define   SW_EXEC     IO.PDR1.BIT.B4    // 実行スイッチ
#define   SW_UP       IO.PDR1.BIT.B5    // モード選択スイッチ+
#define   SW_DOWN     IO.PDR1.BIT.B6    // モード選択スイッチ-
#define   MOTOR_EN    IO.PDR8.BIT.B0    // モータドライバスイッチ
#define   L_MOT_MODE  IO.PDR8.BIT.B3    // 左モータの回転方向
#define   R_MOT_MODE  IO.PDR8.BIT.B4    // 右モータの回転方向

//-------------------------------------------------------------------------
//  マクロ定義
//-------------------------------------------------------------------------
// スイッチ関連
#define   SW_ON       0    // スイッチON (Active Low)
#define   SW_OFF      1    // スイッチOFF
#define   KEY_OFF   200    // スイッチ用チャタリングキャンセル時間
// モード関連
#define   ModeMax     8    // 動作モード数
#define   DISP        0    // モード表示
#define   EXEC        1    // モード実行
// センサ関連
#define   LED_ON      1    // センサ用LED点燈
#define   LED_OFF     0    // センサ用LED消灯
// モータ関連
#define   LeftGo      1    // 左モータ前進
#define   LeftBack    0    // 左モータ後進
#define   RightGo     0    // 右モータ前進
#define   RightBack   1    // 右モータ後進
// 探索関連
#define   GOAL_X      3    // ゴール x座標  フルは 7,7 から 8,8 
#define   GOAL_Y      5    // ゴール y座標 
#define   S_MODE      0    // Search Mode : 未探索区間は壁無しとして扱う
#define   T_MODE      1    // Try Mode    : 未探索区間は壁有りとして扱う

//---------------------------------------------------------------
//  グローバル変数定義
//---------------------------------------------------------------
vushort  wait_timer = 0;   // 内部時計( msec ) : wait関数用カウンタ
ushort   SENSOR_PT;        // 割り込み回数カウント用ポインタ
int      MODE = 0;         // 現在モード格納用
vshort   Batt;             // 電池の電圧
//ブザー関連 : 休符,ド,ド#,レ,レ#,ミ,ファ,ファ#,ソ,ソ#,ラ,ラ#,シ,ド
unsigned char beep_data[14]
 = { 0, 149, 141, 133, 126, 118, 112, 106, 100, 94, 89, 84, 79, 75 };
// センサの事前値格納用
vushort  R_PRE;            // 右センサの値
vushort  L_PRE;            // 左センサの値
vushort  F_PRE;            // 前センサの値
// センサの現在値格納用
vushort  R_SEN;            // 右センサの値
vushort  L_SEN;            // 左センサの値
vushort  F_SEN;            // 前センサの値
// センサのON/OFF用
short    R_SW;             // 右センサのスイッチ
short    L_SW;             // 左センサのスイッチ
short    F_SW;             // 前センサのスイッチ
// センサのしきい値
short    R_REF;            // 右センサしきい値
short    L_REF;            // 左センサしきい値
// 壁の有無判定用しきい値
short    R_LIM;            // 右壁有無しきい値
short    L_LIM;            // 左壁有無しきい値
short    F_LIM;            // 前壁有無しきい値
// モータ関連
ushort   timerL;           // 左タイマー設定値
ushort   timerR;           // 右タイマー設定値
short    ldir;             // 左モータ回転方向
short    rdir;             // 右モータ回転方向
short    speed;            // 目標速度
short    speed_now;        // 現在速度
short    MotorTimer;       // モータ電源コントロールタイマー
short    control_mode;     // 姿勢制御モード  0:なし  1:あり
// 走行関連
short    STEP;             // モータのステップ数
short    GO_STEP;          // 1区間のステップ数
short    TURN_STEP;        // 超信旋回ステップ数
// 探索関連
uchar    head;             // マウスの進行方向 0:北 1:東 2:南 3:西
uchar    head_change;      // 進行方向更新用変数 0:前 1:右 2:後 3:左
uchar    pos_x;            // マウスの現在座標 x
uchar    pos_y;            // マウスの現在座標 y
uchar    map[16][16];      // MAPデータ
uchar    p_map[16][16];    // ポテンシャルMAPデータ

//---------------------------------------------------------------
//  関数プロトタイプ宣言
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
//  メインプログラム
//---------------------------------------------------------------
void main( void )
{
  IO_init();    // H8の初期化
  LCD_init();   // LCDの初期化
  CPU_LED = 1;  // CPU層LEDを消灯

  // 起動音
  beep( 3, 150 );
  beep( 8, 150 );

  // タイトル表示
//  LCD_print( 0, "Sample21" );
  LCD_print( 0, "Samp21b" );  //2018/12/21
  
  // 電圧表示
  LCD_print( 8, "   .  v " );
  LCD_dec_out( 9, Batt/100, 1);     // 十の位を表示
  Batt %= 100;                      // 十の位を削除
  LCD_dec_out(10, Batt/10 , 1);     // 一の位を表示
  Batt %= 10;                       // 一の位を削除
  LCD_dec_out(12, Batt    , 1);     // 残った小数値を表示
  pause( 2000 );

  clear_map();                     // MAPデータ初期化
  load_param();                    // 各種パラメータを読み込み
  change_mode( 0 );                // まず初期画面にする = Mode0

  // メインループ
  while( 1 ){
    if( SW_UP == SW_ON ){          // 上SWが押されている場合
      WaitKeyOff();                // チャタリング防止処理
      change_mode(+1);             // モード+1
    }else if( SW_DOWN == SW_ON ){  // 下SWが押されている場合
      WaitKeyOff();                // チャタリング防止処理
      change_mode(-1);             // モード-1
    }else if( SW_EXEC == SW_ON ){  // 実行SWが押されている場合
      beep( 1, 150 );              // 実行音 : ド
      WaitKeyOff();                // チャタリング防止処理
      exec_mode();                 // モード実行
      MODE = 0;
      change_mode( 0 );            // 実行後は初期画面に戻す
    }
    
    // モードが0ならセンサデータをLCD表示
    if( MODE == 0 )
    {
      LCD_dec_out(  3, F_SEN, 3 ); // 前センサ値をLCD上中央に表示
      LCD_dec_out(  9, L_SEN, 3 ); // 左センサ値をLCD左下に表示
      LCD_dec_out( 13, R_SEN, 3 ); // 右センサ値をLCD右下に表示
    }
  }
}

//---------------------------------------------------------------
//  H8初期化
//---------------------------------------------------------------
void IO_init( void )
{
  // ===== I/O ポートの初期化 =====           ピン番号  7654 3210
  IO.PCR2 = 0x01;       // P20を出力, 未使用は入力      0000 0001
  IO.PCR5 = 0x3f;       // P50-P55を出力, 未使用は入力  0011 1111
  IO.PCR1 = 0x01;       // P14-P16を入力, P10を出力     0000 0001
  IO.PUCR1.BYTE = 0x70; // P14-16プルアップ 0:OFF/1:ON  0111 0000
  IO.PCR8 = 0x1f;       // P80-P84 を出力ポートに設定   0001 1111

  // ===== バッテリー電圧計測 =====
  // ===== A/D 設定 =====
  // AD.ADCSR.BYTE 解説
  // 7:END Flg  6:割り込み(1:ON) 5:変換スタート(1) 4:(0)単一(1)Scan
  // 3:Clock(0)134state / (1)70state<高速変換>
  // 210: 単一モード時 AN***
  //      scanモード時 常に指定したA/D変換をし続ける
  //                   0:AN0,1:AN0-1,2:AN0-2,3:AN0-3,
  //                   4:AN4,5:AN4-5,6:AN4-6,7:AN4-7
  // 0010 1111 = 0x2f // AN7 単一高速変換
  AD.ADCSR.BYTE = 0x2f;             // AN7 単一高速変換
  while( AD.ADCSR.BIT.ADF == 0 );   // 測定が終了するまで待つ
  Batt = AD.ADDRD >> 7;             // 測定データAN7を取り込む

  // ===== タイマーの初期化 =====
  // Timer V 割り込み設定
  TV.TCRV0.BIT.CCLR   = 1;    // コンペアマッチAでTCNTVクリア
  TV.TCRV0.BIT.CKS    = 0;    // 内部クロック設定 : 0(最初はクロック休止)
  TV.TCSRV.BIT.OS     = 3;    // コンペアマッチAでP76(TMOV)にトグル出力
  TV.TCORA            = 0;    // 周期(音程)の設定レジスタ : 0(最初は休符)

  // Timer W 割り込み設定
  TW.TIERW.BIT.IMIEA  = 1;   // 割り込みAを有効
  TW.TIERW.BIT.IMIEB  = 1;   // 割り込みBを有効
  TW.TIERW.BIT.IMIEC  = 1;   // 割り込みCを有効
  TW.TIERW.BIT.IMIED  = 1;   // 割り込みDを有効
  TW.TCRW.BIT.CKS     = 1;   // 割り込みClockセレクト1/2 : 20M/2=10MHz
  TW.TIOR0.BIT.IOA    = 2;   // コンペアマッチでFTIOAより1を出力
  TW.TIOR0.BIT.IOB    = 2;   // コンペアマッチでFTIOBより1を出力
  TW.TIOR1.BIT.IOC    = 0;   // ピン設定：出力禁止
  TW.TIOR1.BIT.IOD    = 0;   // ピン設定 : 出力禁止
  // 割り込み周期設定
  TW.GRA = 0;                // 左モータ用:最初は停止中なので0
  TW.GRB = 0;                // 右モータ用:最初は停止中なので0
  TW.GRC = 10000;            // モータスピード用:停止中1kHz
  TW.GRD = 2000;             // 割り込み周期設定 : 10MHz/2000=5kHz
                             // 200us割り込み
  EI;                        // 割り込み許可. レジスタ設定後に許可.
  TW.TMRW.BIT.CTS     = 1;   // カウンタスタート

  R_SW = LED_ON;             // 右センサON
  L_SW = LED_ON;             // 左センサON
  F_SW = LED_ON;             // 前センサON
}

//---------------------------------------------------------------
//  パラメータ読み込み
//---------------------------------------------------------------
void load_param( void )
{
  // センサしきい値の決め打ち
  R_REF   = 0;    // 区画中央での右センサ値  :2018/12/21
  L_REF   = 0;    // 区画中央での左センサ値  :2018/12/21
  // 壁の有無判定用しきい値:各センサ壁あり最小値と壁なし値の中間値
  R_LIM   = 0;    // 右   :2018/12/21
  L_LIM   = 0;    // 左   :2018/12/21
  F_LIM   = 0;    // 前   :2018/12/21
  // 走行パラメータ
  GO_STEP   = 2450; // 1区間前進ステップ数
  TURN_STEP = 650;  // 旋回ステップ数
}

//---------------------------------------------------------------
//  Timer W 割り込み(200us毎にこの関数が勝手に優先して実行される)
//---------------------------------------------------------------
void int_timerw( void )
{
  int err_l, err_r;
  ushort acc_num, lspeed, rspeed;

  // 左モータ割り込み
  if( TW.TSRW.BIT.IMFA == 1 )
  {
    TW.TSRW.BIT.IMFA = 0;                    // 割り込みフラグクリア
    TW.GRA += timerL;                        // 次の割り込み時間をセットする
    if( speed != 0 )                         // 停止ならパルスをださない
    {
      TW.TCRW.BIT.TOA = 0;
      STEP++;                                // 距離カウンタ更新
    }
    if( ldir == 0 )  L_MOT_MODE = LeftGo;    // 正転
    else             L_MOT_MODE = LeftBack;  // 反転
  }

  // 右モータ割り込み
  if( TW.TSRW.BIT.IMFB == 1 )
  {
    TW.TSRW.BIT.IMFB = 0;                    // 割り込みフラグクリア
    TW.GRB += timerR;                        // 次の割り込み時間をセットする
    if( speed != 0 )                         // 停止ならパルスをださない
    {
      TW.TCRW.BIT.TOB = 0;
      STEP++;                                // 距離カウンタ更新
    }
    if( rdir == 0 )  R_MOT_MODE = RightGo;   // 正転
    else             R_MOT_MODE = RightBack; // 反転
  }

  // モータスピード割り込み
  if( TW.TSRW.BIT.IMFC == 1 )
  {
    TW.TSRW.BIT.IMFC = 0;                    // 割り込みフラグクリア
    // モータの加速処理
    if( speed == 0 )                         // モータ停止中の処理
    {
      speed_now = 0;                         // 速度を0にする
      timerL = 10000;                        // 割り込み周期を1msにする
      timerR = 10000;
      TW.GRA = TW.TCNT + timerL;
      TW.GRB = TW.TCNT + timerR;
      TW.GRC += 10000;
    }else{
      if( speed > speed_now )       speed_now++;  // 加速
      else if( speed < speed_now )  speed_now--;  // 減速
      if( speed_now >= 500 ) speed_now = 499;     // 最高速度
      if( speed_now < 0     ) speed_now = 0;      // 最低速度

      acc_num = AccTable[ speed_now ];     // 加速度テーブルから値取得
      TW.GRC += 10000000L / (unsigned long) acc_num; // 割り込み周期を計算

      // 姿勢制御
      if( control_mode == 1 )
      {
        // 偏差を計算
        err_l = L_SEN - L_REF;  // 左偏差を計算
        err_r = R_SEN - R_REF;  // 右偏差を計算
        // 壁情報から偏差を加工
        if( L_SEN > L_LIM || R_SEN > R_LIM )
        {
          // どちらかに壁がある:偏差が大きい側を優先して補正
          if( err_l > err_r )
            err_r = -1 * err_l;
          else
            err_l = -1 * err_r;
        }else
        {
          // 両方壁なし:補正なし
          err_l = 0;
          err_r = 0;
        }
        // 偏差を用いて補正
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
  {                              // TimerW-D 割り込み
    TW.TSRW.BIT.IMFD = 0;        // 割り込みフラグクリア
                                 // 忘れると次の割り込みがかからない
    TW.GRD += 2000;              // 次の割り込みを再度200usec後に設定

    SENSOR_PT++;                 // タスクポインタの更新
    if( SENSOR_PT == 5 ) SENSOR_PT = 0;  // 0-4の5カウント:200us*5=1ms
                                 // 各処理は1ms周期で実行される
    switch( SENSOR_PT )          // タスクポインタに従って処理を行う
    {
      case 0:  // 1msecタイマー&LCDの更新
               wait_timer++;     // wait関数用カウンタ
               LCD();            // LCD更新処理
               break;

      case 1:  // 右センサ消灯時の測定 P11,AN0(A)
               if ( R_SW == LED_OFF ) break;         // センサON/OFFの確認
               AD.ADCSR.BYTE = 0x28;                 // 反射光の測定開始:AN0単一SCAN
               while( AD.ADCSR.BIT.ADF == 0 );       // 測定が終了するまで待つ
               R_PRE = AD.ADDRA >> 6;       // 測定データを取り込む 0 - 1023(max)
               // 右センサ点灯時の測定 P11,AN0(A)
               LED = LED_ON;                         // LEDを点灯
               LCD_wait(10);                         // しばらく待つ
               AD.ADCSR.BYTE = 0x28;                 // 反射光の測定開始:AN0単一SCAN
               while( AD.ADCSR.BIT.ADF == 0 );       // 測定が終了するまで待つ
               LED = LED_OFF;                        // LEDを消灯
               R_PRE = ( AD.ADDRA >> 6 ) - R_PRE;    // 測定データを取り込む
                                                     // 事前計測値との差分を取る(ノイズ処理)
               if( R_PRE <= 999 )  R_SEN = R_PRE;    // 表示上限処理
               else                R_SEN = 0; 
               break;

      case 2:  // 左センサ消灯時の測定 P11,AN1(B)
               if ( L_SW == LED_OFF ) break;         // センサON/OFFの確認
               AD.ADCSR.BYTE = 0x29;                 // 反射光の測定開始:AN1単一SCAN
               while( AD.ADCSR.BIT.ADF == 0 );       // 測定が終了するまで待つ
               L_PRE = AD.ADDRB >> 6;                // 測定データを取り込む 0 - 1023(max)
               // 左センサ点灯時の測定 P11,AN1(B)
               LED = LED_ON;                         // LEDを点灯
               LCD_wait(10);                         // しばらく待つ
               AD.ADCSR.BYTE = 0x29;                 // 反射光の測定開始:AN1単一SCAN
               while( AD.ADCSR.BIT.ADF == 0 );       // 測定が終了するまで待つ
               LED = LED_OFF;                        // LEDを消灯
               L_PRE = ( AD.ADDRB >> 6 ) - L_PRE;    // 測定データを取り込む
                                                     // 事前計測値との差分を取る(ノイズ処理)
               if( L_PRE <= 999 )  L_SEN = L_PRE;    // 表示上限処理
               else                L_SEN = 0;
               break;

      case 3:  // 前センサ消灯時の測定 P11,AN2(C)
               if ( F_SW == LED_OFF ) break;         // センサON/OFFの確認
               AD.ADCSR.BYTE = 0x2A;                 // 反射光の測定開始:AN2単一SCAN
               while( AD.ADCSR.BIT.ADF == 0 );       // 測定が終了するまで待つ
               F_PRE = AD.ADDRC >> 6;        // 測定データを取り込む 0 - 1023(max)
               // 前センサ点灯時の測定 P11,AN2(C)
               LED = LED_ON;                         // LEDを点灯
               LCD_wait(10);                         // しばらく待つ
               AD.ADCSR.BYTE = 0x2A;                 // 反射光の測定開始:AN2単一SCAN
               while( AD.ADCSR.BIT.ADF == 0 );       // 測定が終了するまで待つ
               LED = LED_OFF;                        // LEDを消灯
               F_PRE = ( AD.ADDRC >> 6 ) - F_PRE;    // 測定データを取り込む
                                                     // 事前計測値との差分を取る(ノイズ処理)
               if( F_PRE <= 999 )  F_SEN = F_PRE;
               else                F_SEN = 0;        // 表示上限処理
               break;

      case 4:  // モータ用電源コントロール
               if( speed != 0 ) MotorTimer = 3000;   // モータ動作時はタイマーセット
               else             MotorTimer--;        // モータ停止時はカウントダウン
               if( MotorTimer < 0 )  MotorTimer =  0;
               // モータを動かさない時は電源をOFF(モータ停止から3秒後)
               if( MotorTimer == 0 )  MOTOR_EN   =  0;  // OFF
               else                   MOTOR_EN   =  1;  // ON
               break;

      default: break;
    }
  }
}

//---------------------------------------------------------------
//  wait関数(1msタイマー)
//---------------------------------------------------------------
void pause( int x )
{
  wait_timer = 0;
  while( wait_timer != x ); // 終了時間まで待つ
}

//-------------------------------------------------------------------------
//  キーオフ処理
//-------------------------------------------------------------------------
void WaitKeyOff( void )
{
  // チャタリング防止処理
  pause( KEY_OFF );             // 設定した時間([ms])待つ
  // 全てのスイッチがOFFになるまでループして待つ
  while(( SW_UP == SW_ON )||( SW_DOWN == SW_ON )||( SW_EXEC == SW_ON ));
}

//------------------------------------------------------------------------
// ビープ音
//------------------------------------------------------------------------
void beep(unsigned char tone,int value)
{
  TV.TCRV0.BIT.CKS  = 3;                  // 内部クロックφ/128で動作開始
  TV.TCORA          = beep_data[ tone ];  // 音程の設定
  pause( value );                         // Beep音の長さ
  TV.TCRV0.BIT.CKS  = 0;                  // Beep動作停止
}

//-------------------------------------------------------------------------
//  モード表示
//-------------------------------------------------------------------------
void change_mode( int x )
{
  MODE += x;                            // モード更新
  if( MODE >= ModeMax ) MODE = 0;       // モードが超えている場合は0に戻す
  if( MODE < 0 )  MODE = ModeMax - 1;   // モードが負の場合はモードを最大値に設定

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
//  モード処理
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
//  Mode0 : センサチェック
//-------------------------------------------------------------------------
void mode0( int x )
{
  if( x == DISP )  // DISPモードの場合
  {
    // モード内容表示
    LCD_print( 0, "0:Sensor" );
    LCD_print( 8, "        " );
    pause( 1000 );
    LCD_print( 0, "  F     " );
    LCD_print( 8, "L   R   " );
    return;                     // 以下の実行処理をしないで戻る
  }

  // 実行モードの場合
  L_REF = 0; R_REF = 0;         // データ初期化

  for( x = 0; x < 32; x++ )     // データ測定(32ポイント)
  {
    L_REF += L_SEN;
    R_REF += R_SEN;
    pause(1);
  }

  R_REF = R_REF / 32;           // 測定データを平均化
  L_REF = L_REF / 32;

  LCD_print( 0, " L    R " );
  LCD_print( 8, "        " );
  LCD_dec_out(  8, L_REF, 3 );  // 左センサ値をLCDに表示
  LCD_dec_out( 13, R_REF, 3 );  // 右センサ値をLCDに表示

  pause( 2000 );                // 2秒間表示
}

//-------------------------------------------------------------------------
//  Mode1 : モータテスト
//-------------------------------------------------------------------------
void mode1(int x)
{
  if( x == DISP )  // DISPモードの場合
  {
    // モード内容表示
    LCD_print( 0, "1:M-TEST" );
    LCD_print( 8, "        " );
    return;                     // 以下の実行処理をしないで戻る
  }

  // 実行モードの場合
  LCD_print( 8,"SPD=     ");
  rdir = 0; ldir = 0;           // 回転方向を直進
  control_mode = 1;             // 直線走行用姿勢制御あり

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
//  Mode2 : 1区間前進
//-------------------------------------------------------------------------
void mode2(int x)
{
  if( x == DISP )  // DISPモードの場合
  {
    // モード内容表示
    LCD_print( 0, "2: 1 GO " );
    LCD_print( 8, "STEP    " );
    return;                     // 以下の実行処理をしないで戻る
  }

  // 実行モードの場合
  while(1){
    LCD_dec_out( 12, GO_STEP, 4 );
    if( SW_UP   == 0 ) { GO_STEP += 10; WaitKeyOff(); }
    if( SW_DOWN == 0 ) { GO_STEP -= 10; WaitKeyOff(); }
    if( SW_EXEC == 0 ) { WaitKeyOff();  com_go( 1 );  com_stop(); }
  }
}

//-------------------------------------------------------------------------
//  Mode3 : N区間前進
//-------------------------------------------------------------------------
void mode3(int x)
{
  int n = 5;                    // 前進区間数：初期値 5
  if( x == DISP )  // DISPモードの場合
  {
    // モード内容表示
    LCD_print( 0, "3: N GO " );
    LCD_print( 8, "     N  " );
    return;                     // 以下の実行処理をしないで戻る
  }

  // 実行モードの場合
  LCD_dec_out( 8, GO_STEP, 4 );
  while(1){
    LCD_dec_out( 14, n, 2 );
    if( SW_UP   == 0 ) { n++; WaitKeyOff(); }
    if( SW_DOWN == 0 ) { n--; WaitKeyOff(); }
    if( SW_EXEC == 0 ) { WaitKeyOff();  com_go( n );  com_stop(); }
  }
}

//-------------------------------------------------------------------------
//  Mode4 : 180ターンR
//-------------------------------------------------------------------------
void mode4( int x )
{
  if( x == DISP )  // DISPモードの場合
  {
    // モード内容表示
    LCD_print( 0, "4: TURN " );
    LCD_print( 8, "        " );
    return;                     // 以下の実行処理をしないで戻る
  }

  // 実行モードの場合
  while(1){
    LCD_dec_out( 10, TURN_STEP, 4 );
    if( SW_UP   == 0 ) { TURN_STEP += 10; WaitKeyOff(); }
    if( SW_DOWN == 0 ) { TURN_STEP -= 10; WaitKeyOff(); }
    if( SW_EXEC == 0 ) { com_turn(2); com_stop(); }
  }
}

//-------------------------------------------------------------------------
//  Mode5 : 探索走行
//-------------------------------------------------------------------------
void mode5( int x )
{
  if( x == DISP )  // DISPモードの場合
  {
    // モード内容表示
    LCD_print( 0, "5:Search" );
    LCD_print( 8, "Spd  200" );
    return;                     // 以下の実行処理をしないで戻る
  }

  // 実行モードの場合
  pos_x = 0; pos_y = 0; head = 0;
  // 往復探索
  mouse_search( GOAL_X, GOAL_Y, 200, S_MODE );  // 行きの探索
  mouse_search( 0, 0, 200, S_MODE );            // 帰りの探索
}

//-------------------------------------------------------------------------
//  Mode6 : 二次走行
//-------------------------------------------------------------------------
void mode6( int x )
{
  if( x == DISP )  // DISPモードの場合
  {
    // モード内容表示
    LCD_print( 0, "6:Try   " );
    LCD_print( 8, "Spd  200" );
    return;                     // 以下の実行処理をしないで戻る
  }

  // 実行モードの場合
  // 二次走行
  pos_x = 0; pos_y = 0; head = 0;
  mouse_search( GOAL_X, GOAL_Y, 200, T_MODE );
}

//-------------------------------------------------------------------------
//  Mode7 : myself
//-------------------------------------------------------------------------
void mode7( int x )
{
  if( x == DISP )  // DISPモードの場合
  {
    // モード内容表示
    LCD_print( 0, "7:myself" );
    LCD_print( 8, "--------" );
    return;                     // 以下の実行処理をしないで戻る
  }

  // 実行モードの場合
  com_back(1);
}

//-------------------------------------------------------------------------
//  探索関数
//-------------------------------------------------------------------------
void mouse_search( int goal_x, int goal_y, int spd, int mode )
{
  short motion;
  countdown();                  // カウントダウン

  while( 1 ){
    // １つのループは区間中心から次の区間中心まで
    // 最初に半区画直進
    control_mode = 1;             // 姿勢制御ON
    rdir = 0; ldir = 0;           // 回転方向を直進
    STEP = 0;                     // 距離カウンタリセット
    speed = spd;                  // 速度設定

    // 座標更新
    if     ( head == 0 ) pos_y++; // 北向き y+1
    else if( head == 1 ) pos_x++; // 東向き x+1
    else if( head == 2 ) pos_y--; // 南向き y-1
    else if( head == 3 ) pos_x--; // 西向き x-1
    
    // ポテンシャルMAP計算
    make_potential( goal_x, goal_y, mode );
    
    while( STEP < GO_STEP / 2 );  // 半区間進む

    // 柱まで進んだら
    // 壁情報取得＆MAPデータ上書き（探索走行時のみ）
    if( mode == S_MODE )
      make_map_data(); 

    // 足立法で行動決定
    motion = search_adachi();
    
    // ゴール時の例外処理（上で決めた行動が上書きされる）
    if( pos_x == goal_x && pos_y == goal_y )
      motion = 4;                       // ゴール到達：反転停止

    // 行動を実行
    switch( motion ){
      // 直進
      case  0 : while( STEP < GO_STEP );  // 残り半区間進む
                head_change = 0;          // 進行方向更新変数を前に設定
                break;
      // 右折
      case  1 : while( STEP < GO_STEP - speed_now * 2 );  // 減速域を残して直進
                speed = 1;
                while( STEP < GO_STEP );  // 残りステップ数で減速
                com_turn( 0 );            // 右90度旋回
                head_change = 1;          // 進行方向更新変数を右に設定
                break;
      // 反転
      case  2 : while( STEP < GO_STEP - speed_now * 2 );  // 減速域を残して直進
                speed = 1;
                while( STEP < GO_STEP );  // 残りステップ数で減速
                com_turn( 2 );            // 反転
                head_change = 2;          // 進行方向更新変数を後に設定
                break;
      // 左折
      case  3 : while( STEP < GO_STEP - speed_now * 2 );  // 減速域を残して直進
                speed = 1;
                while( STEP < GO_STEP );  // 残りステップ数で減速
                com_turn( 1 );            // 左90度旋回
                head_change = 3;          // 進行方向更新変数を左に設定
                break;
      // 反転停止
      case  4 : while( STEP < GO_STEP - speed_now * 2 );  // 減速域を残して直進
                speed = 1;
                while( STEP < GO_STEP );  // 残りステップ数で減速
                com_turn( 2 );            // 反転
                com_stop();               // 停止
                head_change = 2;          // 進行方向更新変数を後に設定
                head = ( head + head_change ) & 0x03; // 詳細は下を参照
                finish();                 // ゴール音
                return;                   // ループ終了
                break;
      // その他
      default : com_stop();               // 停止
                head_change = 0;          // 進行方向更新変数を前に設定
                head = ( head + head_change ) & 0x03; // 詳細は下を参照
                return;                   // ループ終了
                break;
    }
    
    // 進行方向更新変数head_changeを用いて現在の進行方向headを更新
    head = ( head + head_change ) & 0x03; // 更新数値を加算して2進数下2桁でマスク
                                          // 00 -> 01 -> 10 -> 11 -(マスク)-> 00
  }
}

//-------------------------------------------------------------------------
//  直進モジュール (N区間前進)
//-------------------------------------------------------------------------
void com_go( int n )
{
  control_mode = 1;                       // 直線走行用姿勢制御
  STEP = 0;                               // 距離カウンタクリア
  rdir = 0; ldir = 0;                     // 回転方向を直進

  // 加速モード
  speed = 200;        // 目標速度設定
  while( speed > speed_now );                   // 目標速度になるまで加速
  // 定速モード
  speed = speed_now;  // 加速後の速度
  while( STEP < GO_STEP * n - speed_now * 2 );  // 減速ステップ数を残して定速移動
                                                // 全体ステップ数-減速用ステップ数
  // 減速モード
  speed = 1;          // 最低速度設定
  while( STEP < GO_STEP * n );                  // 残りのステップ数で減速
}

//-------------------------------------------------------------------------
//  後退モジュール (N区間後退)
//-------------------------------------------------------------------------
void com_go( int n )
{
  control_mode = 1;                       // 直線走行用姿勢制御
  STEP = 0;                               // 距離カウンタクリア
  rdir = 1; ldir = 1;                     // 回転方向を直進

  // 加速モード
  speed = 200;        // 目標速度設定
  while( speed > speed_now );                   // 目標速度になるまで加速
  // 定速モード
  speed = speed_now;  // 加速後の速度
  while( STEP < GO_STEP * n - speed_now * 2 );  // 減速ステップ数を残して定速移動
                                                // 全体ステップ数-減速用ステップ数
  // 減速モード
  speed = 1;          // 最低速度設定
  while( STEP < GO_STEP * n );                  // 残りのステップ数で減速
}

//-------------------------------------------------------------------------
//  停止モジュール
//-------------------------------------------------------------------------
void com_stop( void )
{
  control_mode = 0;           // 姿勢制御無し
  rdir = 0; ldir = 0;         // モータの回転方向を前進
  STEP = 0;                   // 距離カウンタをリセット
  speed = 0;  speed_now = 0;  // モータの制御用の変数をリセット
  pause(100);                 // 0.1秒モータを停止
}

//-------------------------------------------------------------------------
//  旋回モジュール (0:R90 1:L90 2:R180 3:L180)
//-------------------------------------------------------------------------
void com_turn( int t_mode )
{
  short T_STEP;

  com_stop();                                             // 停止
  control_mode = 0;                                       // 姿勢制御なし
  if     ( t_mode == 0 ) { T_STEP = TURN_STEP; rdir = 1; ldir = 0; } // 右９０度
  else if( t_mode == 1 ) { T_STEP = TURN_STEP; rdir = 0; ldir = 1; } // 左９０度
  else if( t_mode == 2 ) { T_STEP = TURN_STEP * 2; rdir = 1; ldir = 0; } // 右反転
  else if( t_mode == 3 ) { T_STEP = TURN_STEP * 2; rdir = 0; ldir = 1; } // 左反転

  // 加速モード
  speed = 100;        // 目標速度設定
  while( speed > speed_now );                   // 目標速度になるまで加速
  // 定速モード
  speed = speed_now;  // 加速後の速度
  while( STEP < T_STEP - speed_now * 2 );       // 減速ステップ数を残して定速移動
                                                // 全体ステップ数-減速用ステップ数
  // 減速モード
  speed = 1;          // 最低速度設定
  while( STEP < T_STEP );                       // 残りのステップ数で減速
}

//-------------------------------------------------------------------------
//  カウントダウン
//-------------------------------------------------------------------------
void countdown( void )
{
  beep( 0, 850 );
  beep( 1, 150 );
  beep( 0, 850 );
  
  R_SW = LED_OFF;        // 右センサOFF
  L_SW = LED_OFF;        // 左センサOFF
  F_SW = LED_OFF;        // 前センサOFF
  beep( 1, 150 );
  beep( 0, 850 );

  R_SW = LED_ON;         // 右センサON
  L_SW = LED_ON;         // 左センサON
  F_SW = LED_ON;         // 前センサON
  beep( 13, 1000 );
}

//-------------------------------------------------------------------------
//  ゴール音
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
//  壁のセンシング
//-------------------------------------------------------------------------
int get_wall_data( void )
{
  short wall;

  // センサデータを入力，閾値と比較して壁の有無を判定
  wall = 0;
  if( F_SEN > F_LIM )  wall |= 0x01; // 前壁あり
  if( R_SEN > R_LIM )  wall |= 0x02; // 右壁あり
  if( L_SEN > L_LIM )  wall |= 0x08; // 左壁あり
  // 後壁はあるわけないので見ない

  return( wall );
}

//-------------------------------------------------------------------------
// MAPデータの見方
// 探索記録 bit 7 6 5 4 = 西 南 東 北 / 値 = 1:既探索 0:未探索
// 壁情報   bit 3 2 1 0 = 西 南 東 北 / 値 = 1:壁有り 0:壁無し
//-------------------------------------------------------------------------

//-------------------------------------------------------------------------
//  MAPデータ初期化
//-------------------------------------------------------------------------
void clear_map( void )
{
  int x, y;
  // 全ての区間を壁なし＆未探索に初期化
  for( y = 0 ; y < 16 ; y++ )
    for( x = 0 ; x < 16 ; x++ )
      map[ x ][ y ] = 0x00;

  // 西側の外壁（x=0，y=0～15）を上書き
  for( y = 0 ; y < 16 ; y++ )
    map[ 0 ][ y ] = 0x88;  // 西のみ既探索(8)＆西のみ壁あり(8)
  // 南側の外壁（x=0～15，y=0）を上書き
  for( x = 0 ; x < 16 ; x++ )
    map[ x ][ 0 ] = 0x44;  // 南のみ既探索(4)＆南のみ壁あり(4)
  // 東側の外壁（x=15，y=0～15）を上書き
  for( y = 0 ; y < 16 ; y++ )
    map[ 15 ][ y ] = 0x22; // 東のみ既探索(2)＆東のみ壁あり(2)
  // 北側の外壁（x=0～15，y=15）を上書き
  for( x = 0 ; x < 16 ; x++ )
    map[ x ][ 15 ] = 0x11; // 北のみ既探索(1)＆北のみ壁あり(1)

  // スタート区間（x=0，y=0）の上書き
  map[ 0 ][ 0 ] = 0xfe;  // 西南東北既探索(8+4+2+1=f)＆西南東壁あり(8+4+2=e)
  // スタート区間1つ右（x=1，y=0）の上書き
  map[ 1 ][ 0 ] = 0xcc;  // 西南既探索(8+4=c)＆西南壁あり(8+4=c)

  // 左上角（x=0，y=15）の上書き
  map[ 0 ][ 15 ] = 0x99; // 西北既探索(8+1=9)＆西北壁あり(8+1=9)
  // 右下角（x=15，y=0）の上書き
  map[ 15 ][ 0 ] = 0x66; // 南東既探索(4+2=6)＆南東壁あり(4+2=6)
  // 右上角（x=15，y=15）の上書き
  map[ 15 ][ 15 ] = 0x33;// 東北既探索(2+1=3)＆東北壁あり(2+1=3)
}

//-------------------------------------------------------------------------
//  センサ情報から探索記録＆壁情報をMAPデータに更新
//-------------------------------------------------------------------------
void make_map_data( void )
{
  uchar wall;

  // 壁情報取得
  wall = get_wall_data();
  // 方向合わせ処理のために上位4bitに下位4bitの壁情報をコピー
  wall = ( wall & 0x0f ) | ( wall << 4 );
  // マウスの進行方向にあわせて壁データを加工
  if     ( head == 1 ) wall = wall >> 3; // 北が前の情報を東が前に加工
  else if( head == 2 ) wall = wall >> 2; // 北が前の情報を南が前に加工
  else if( head == 3 ) wall = wall >> 1; // 北が前の情報を西が前に加工
  // 西南東北を探索済みにする
  wall |= 0xf0;
  // 壁情報をMAPデータに上書き
  map[ pos_x ][ pos_y ] = wall;

  // 西南東北の隣区画のMAPデータを上書き

  // 現在区画の東壁情報を1つ右区画の西壁情報として上書きする処理
  // ここだけ詳細に説明．残り3つ（下，左，上）はまとめて記述
  if( pos_x != 15 ){  // 一番東側の区画の時以外
    // 右区画の西側情報（探索記録＆壁情報）を消去
    map[ pos_x + 1 ][ pos_y ] &= 0x77;
    // 右区画の西側探索記録を既探索とする
    map[ pos_x + 1 ][ pos_y ] |= 0x80;
    // 現在区画の東側情報を西側情報に変換して右区画の西側壁情報に上書き
    map[ pos_x + 1 ][ pos_y ] |= ( map[ pos_x ][ pos_y ] << 2 ) & 0x08;
  }
  
  // 現在区画の南壁情報を1つ下区画の北壁情報として上書きする処理
  if(pos_y!=0) map[pos_x][pos_y-1]=(map[pos_x][pos_y-1]&0xee)|0x10|((wall>>2)&0x01);
  // 現在区画の西壁情報を1つ左区画の東壁情報として上書きする処理
  if(pos_x!=0) map[pos_x-1][pos_y]=(map[pos_x-1][pos_y]&0xdd)|0x20|((wall>>2)&0x02);
  // 現在区画の北壁情報を1つ上区画の南壁情報として上書きする処理
  if(pos_y!=15)map[pos_x][pos_y+1]=(map[pos_x][pos_y+1]&0xbb)|0x40|((wall<<2)&0x04);
}

//-------------------------------------------------------------------------
//  等高線（ポテンシャル場）作成
//-------------------------------------------------------------------------
void make_potential( int gx, int gy, int mode )
{
  uchar check_num, flg;
  uchar x,y;

  // ポテンシャルMAP初期化(全て最大値255にする)
  for( y = 0 ; y < 16 ; y++ )
    for( x = 0 ; x < 16 ; x++ )
      p_map[ x ][ y ] = 255;

  // ゴール座標にポテンシャル0を書き込む
  p_map[ gx ][ gy ] = 0;

  check_num = 0;
  do{
    flg = 0;  // 変更フラグ初期化
    for( y = 0 ; y < 16 ; y++ ){
      for( x = 0 ; x < 16 ; x++ ){
        if( p_map[ x ][ y ] == check_num ){  // 今回対象区画とするポテンシャル
          if( mode == S_MODE ){

            // 探索走行(Search Mode)
            // 北側の壁がない場合：北側のポテンシャルを対象区画のポテンシャルより+1
            if((( map[ x ][ y ] & 0x01 ) == 0 ) && ( y != 15 )){
              if( p_map[ x ][ y + 1 ] == 255 ){// まだポテンシャルを書いてなければ
                p_map[ x ][ y + 1 ] = check_num + 1;
                flg = 1;  // 変更したのでフラグON
              }
            }
            // 東側の壁も同様に処理
            if((( map[ x ][ y ] & 0x02 ) == 0 ) && ( x != 15 ))
              if(p_map[x+1][y]==255){p_map[x+1][y]=check_num+1;flg=1;}
            // 南側の壁も同様に処理
            if((( map[ x ][ y ] & 0x04 ) == 0 ) && ( y != 0 ))
              if(p_map[x][y-1]==255){p_map[x][y-1]=check_num+1;flg=1;}
            // 西側の壁も同様に処理
            if((( map[ x ][ y ] & 0x08 ) == 0 ) && ( x != 0 ))
              if(p_map[x-1][y]==255){p_map[x-1][y]=check_num+1;flg=1;}

          }else{

           // 二次走行(Try Mode)
           // 北側が壁なし＆既探索の場合(壁なしでも未探索はポテンシャル255のまま)
            // 北側のポテンシャルを対象区画のポテンシャルより+1
            if((( map[ x ][ y ] & 0x11 ) == 0x10 ) && ( y != 15 )){
              if( p_map[ x ][ y + 1 ] == 255 ){// まだポテンシャルを書いてなければ
                p_map[ x ][ y + 1 ] = check_num + 1;
                flg = 1;  // 変更したのでフラグON
              }
            }
            // 東側の壁も同様に処理
            if((( map[ x ][ y ] & 0x22 ) == 0x20 ) && ( x != 15 ))
              if(p_map[x+1][y]==255){p_map[x+1][y]=check_num+1;flg=1;}
            // 南側の壁も同様に処理
            if((( map[ x ][ y ] & 0x44 ) == 0x40 ) && ( y != 0 ))
              if(p_map[x][y-1]==255){p_map[x][y-1]=check_num+1;flg=1;}
            // 西側の壁も同様に処理
            if((( map[ x ][ y ] & 0x88 ) == 0x80 ) && ( x != 0 ))
              if(p_map[x-1][y]==255){p_map[x-1][y]=check_num+1;flg=1;}

          }
        }
      }
    }
    check_num++;      // 次のループのために対象ポテンシャルを+1
  }while( flg != 0 ); // 今回のループで変更箇所が無ければ作成完了
}

//-------------------------------------------------------------------------
//  探索：左手法
//-------------------------------------------------------------------------
int search_left_hand( void ){
  short wall_data, motion;

  wall_data = get_wall_data();  // 壁情報取得

  // 壁情報を用いて次の行動を決定（左手法）
    // motion = 0:直進 / 1:右折 / 2:反転 / 3:左折 / 4:反転停止
    switch( wall_data ){
      case  0x00  : motion = 3; break;  // 左に壁なし:左折
      case  0x01  : motion = 3; break;  // 左に壁なし:左折
      case  0x02  : motion = 3; break;  // 左に壁なし:左折
      case  0x03  : motion = 3; break;  // 左に壁なし:左折
      case  0x04  : motion = 3; break;  // 左に壁なし:左折
      case  0x05  : motion = 3; break;  // 左に壁なし:左折
      case  0x06  : motion = 3; break;  // 左に壁なし:左折
      case  0x07  : motion = 3; break;  // 左に壁なし:左折
      case  0x08  : motion = 0; break;  // 左に壁,前に壁なし:直進
      case  0x09  : motion = 1; break;  // 左に壁,前に壁,右に壁なし:右折
      case  0x0a  : motion = 0; break;  // 左に壁,前に壁なし:直進
      case  0x0b  : motion = 2; break;  // 左に壁,前に壁,右に壁:反転
      default     : motion = 4; break;  // 後ろに壁:あり得ないので停止
  }
  return( motion );
}

//-------------------------------------------------------------------------
//  探索：拡張左手法
//-------------------------------------------------------------------------
int search_ex_left_hand( void ){
  short wall_data, motion, val, min_val;

  wall_data = map[ pos_x ][ pos_y ];  // 現在区画の壁情報取得

  min_val = 8;  // 計算される優先度の最大値+1を初期値に設定

  // 北方向の優先度の計算
  if(( wall_data & 0x01 ) == 0 ){     // 北方向に壁が無いとき
    // 1.方向による優先度の計算
    // マウスから見たこの壁の方向と優先度 左:0, 前:1，右:2，後:3
    // head=0（マウスの頭が北）の場合は優先度1（北は前）
    // head=1（マウスの頭が東）の場合は優先度0（北は左）
    // head=2（マウスの頭が南）の場合は優先度3（北は後）
    // head=3（マウスの頭が西）の場合は優先度2（北は右）
    val = (( 3 - head ) + 2 ) & 0x03;

    // 2.未探索／既探索による優先度の計算
    // 未探索:0，既探索:+4
    if(( map[ pos_x ][ pos_y + 1 ] & 0xf0 ) == 0xf0 ) val += 4;

    // 3.この壁が優先かどうかを判断
    // ここまでの計算で以下の優先度のどれかになる
    // 0:北方向がマウスの左側で未探索
    // 1:北方向がマウスの前側で未探索
    // 2:北方向がマウスの右側で未探索
    // 3:北方向がマウスの後側で未探索（※有り得ない）
    // 4:北方向がマウスの左側で既探索
    // 5:北方向がマウスの前側で既探索
    // 6:北方向がマウスの右側で既探索
    // 7:北方向がマウスの後側で既探索

    if( val < min_val ){
      min_val = val;  // 最小値の更新
      motion = 0;     // 移動すべき方向を北に設定
    }
  }

  // 東方向の優先度の計算
  if(( wall_data & 0x02 ) == 0 ){     // 東方向に壁が無いとき
    val = (( 3 - head ) + 3 ) & 0x03;
    if(( map[ pos_x + 1 ][ pos_y ] & 0xf0 ) == 0xf0 ) val += 4;
    if( val < min_val ){
      min_val = val;  // 最小値の更新
      motion = 1;     // 移動すべき方向を東に設定
    }
  }

  // 南方向の優先度の計算
  if(( wall_data & 0x04 ) == 0 ){     // 南方向に壁が無いとき
    val = (( 3 - head ) + 0 ) & 0x03;
    if(( map[ pos_x ][ pos_y - 1 ] & 0xf0 ) == 0xf0 ) val += 4;
    if( val < min_val ){
      min_val = val;  // 最小値の更新
      motion = 2;     // 移動すべき方向を南に設定
    }
  }

  // 西方向の優先度の計算
  if(( wall_data & 0x08 ) == 0 ){     // 西方向に壁が無いとき
    val = (( 3 - head ) + 1 ) & 0x03;
    if(( map[ pos_x - 1 ][ pos_y ] & 0xf0 ) == 0xf0 ) val += 4;
    if( val < min_val ){
      min_val = val;  // 最小値の更新
      motion = 3;     // 移動すべき方向を西に設定
    }
  }

  // 移動すべき方向から行動を決定
  // (目標方向-現在方向)を2進数下2桁でマスク
  // 引き算の結果が負の場合は2の補数でマスクされる
  // 11 -> 10 -> 01 -> 00 -> (-1)=11 -> (-2)=10 ...
  motion = ( motion - head ) & 0x03;

  return( motion );
}

//-------------------------------------------------------------------------
//  探索：足立法
//-------------------------------------------------------------------------
int search_adachi( void )
{
  uchar wall_data, motion;
  short val, min_val;

  // 現在区画の壁情報取得
  wall_data = map[ pos_x ][ pos_y ];

  // 計算される優先度の最大値を初期値に設定
  min_val = 1025;  // 区画ポテンシャル最大値+1 255*4+4 +1 =1025

  // 周囲４つの方向に対して優先度を計算し，
  // 一番優先度が高い（値が小さい）区画に移動する．
  // 優先度はポテンシャル，未／既探索，直進方向の順．
  // 例：ポテンシャルが0の場合＝基本優先度は0*4+4=4
  // ※未探索なら-2，直進なら-1の減算方式
  // 4:既探索＆直進以外
  // 3:既探索＆直進
  // 2:未探索＆直進以外
  // 1:未探索＆直進
  // 優先度が同じ結果の場合は北東南西の順に優先される

  // 北方向の優先度の計算
  if(( wall_data & 0x01 ) == 0 ){     // 北方向に壁が無いとき
    // 1.ポテンシャルを元に基本優先度を計算
    val = p_map[ pos_x ][ pos_y + 1 ] * 4 + 4;
    // 2.方向による優先度の計算
    // 北方向が進行方向だった場合：-1(優先度を1上げる)
    if( head == 0 )  val -= 1;
    // 3.未探索／既探索による優先度の計算
    // 未探索:-2(優先度を2上げる)，既探索:0
    if(( map[ pos_x ][ pos_y + 1 ] & 0xf0 ) != 0xf0 )  val -= 2;
    // 最小値の更新
    if( val < min_val ){
      min_val = val;
      motion = 0;  // 移動すべき方向を北に設定
    }
  }

  // 東方向の優先度の計算
  if(( wall_data & 0x02 ) == 0 ){     // 東方向に壁が無いとき
    val = p_map[ pos_x + 1 ][ pos_y ] * 4 + 4;
    if( head == 1 )  val -= 1;
    if(( map[ pos_x + 1 ][ pos_y ] & 0xf0 ) != 0xf0 )  val -= 2;
    if( val < min_val ){
      min_val = val;
      motion = 1;  // 移動すべき方向を東に設定
    }
  }

  // 南方向の優先度の計算
  if(( wall_data & 0x04 ) == 0 ){     // 南方向に壁が無いとき
    val = p_map[ pos_x ][ pos_y - 1 ] * 4 + 4;
    if( head == 2 )  val -= 1;
    if(( map[ pos_x ][ pos_y - 1 ] & 0xf0 ) != 0xf0 )  val -= 2;
    if( val < min_val ){
      min_val = val;
      motion = 2;  // 移動すべき方向を南に設定
    }
  }

  // 西方向の優先度の計算
  if(( wall_data & 0x08 ) == 0 ){     // 西方向に壁が無いとき
    val = p_map[ pos_x - 1 ][ pos_y ] * 4 + 4;
    if( head == 3 )  val -= 1;
    if(( map[ pos_x - 1 ][ pos_y ] & 0xf0 ) != 0xf0 )  val -= 2;
    if( val < min_val ){
      min_val = val;
      motion = 3;  // 移動すべき方向を西に設定
    }
  }

  // 移動すべき方向から行動を決定
  motion = ( motion - head ) & 0x03;

  return( motion );
}
