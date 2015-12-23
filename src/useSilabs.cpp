#include "useSilabs.h"


int hr;//返り値
HID_UART_DEVICE dev;	//通信ハンドル
short MotorPosition[5] = {0,0,0,0,0};//モータ値で代入
short StartMotorPosition[5] = {0,0,0,0,0};//モータ値で代入
short HomeMotorPosition[5] = {0,0,0,0,0};//原点復帰位置
int servoNum = 5;//サーボ数
int CartesianSpeed = 0;//直交座標空間での速度[%]
int JointSpeed = 0;//関節座標空間での速度[%]

Catesian CatesianLimit = {
	{X_LimitMax,X_LimitMin},
	{Y_LimitMax,Y_LimitMin},
	{Z_LimitMax,Z_LimitMin}
};

JLimit JointLimit[5] = {
	{Angle1_LimitMax,Angle1_LimitMin},
	{Angle2_LimitMax,Angle2_LimitMin},
	{Angle3_LimitMax,Angle3_LimitMin},
	{Angle4_LimitMax,Angle4_LimitMin},
	{Angle5_LimitMax,Angle5_LimitMin}
};

Offset BaseOffset = { 0.0 , 0.0 , 0.0 , 0.0 };//[m][m][m][rad]

/************************************************

	int RSTorqueOnOff( HID_UART_DEVICE dev, short sMode ,BYTE id,int num)
	
	概要：モータのトルクを切り替える関数。IDが連続した複数のモータを同時に設定可能。

	引数：
		HID_UART_DEVICE dev	… 通信ハンドル
		short sMode			… ゲインのON/OFFを指定。0=off、1=ON
		BYTE id				… 切り替えを開始するモータのID（複数のモータを切り替える場合、先頭のモータのID）
		int num				… 切り替えを行うモータ数

	戻り値：
		正常に処理を実行できたらTRUE、そうでなければFALSE

*************************************************/
int RSTorqueOnOff( HID_UART_DEVICE dev, short sMode ,BYTE id,int num )
{
	unsigned char	sendbuf[256],*bufp;				//送信バッファ関係
	unsigned char	sum;							//チェックサム計算用
	int				ret;							//戻り値記録用
	DWORD			data_len=0,len=0;				//パケット長と書き込みサイズ取得用
	unsigned char	i;


	//送信バッファクリア
	memset( sendbuf, 0x00, sizeof( sendbuf ));

	//パケット作成
	//1．ヘッダ・共通部分の作成
	sendbuf[0]  = (unsigned char)0xFA;				// ヘッダー1
	sendbuf[1]  = (unsigned char)0xAF;				// ヘッダー2
	sendbuf[2]  = (unsigned char)0x00;				// サーボID(常に0)
	sendbuf[3]  = (unsigned char)0x00;				// フラグ(常に0)
	sendbuf[4]  = (unsigned char)0x24;				// アドレス(トルクON/OFF 0x24=36)
	sendbuf[5]  = (unsigned char)0x01+1;			// 長さ(1byte)
	sendbuf[6]  = (unsigned char)num;				// モータの個数

	//共通部分のパケット長を記録
	data_len = 7;			

	//2．サーボ個別部分のパケット作成
	bufp = &sendbuf[7];								// 送信バッファの個別メッセージ部分の開始アドレスを代入

	//書き換えるモータの個数分だけ、個別のパケットを追加
	for(i=0;i<num;i++){
		*bufp++ = id+i;								//モータのID
		data_len++;									//パケット長を1byte加算

		*bufp++ = (unsigned char)(sMode&0x00FF);	//ON・OFFフラグ
		data_len++;									//パケット長を1byte加算
	}

	//3．チェックサムの計算
	//チェックサムは、送信バッファの3byte目(サーボID)～終端を1byteずつXORした値です。
	sum = sendbuf[2];
	for( i = 3; i < data_len; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[data_len] = sum;						// 求まったチェックサムを送信バッファの最後に代入
	data_len++;										//パケット長を1byte加算

	//4．メッセージの送信
	ret = HidUart_Write( dev, (BYTE*) sendbuf, data_len, &len );

	if(ret!=HID_UART_SUCCESS) return FALSE;

	//5．ローカルエコーの読み取り
	return ReadLocalEcho(dev,sendbuf,data_len);
}



/************************************************

	int RSGetAngle( HID_UART_DEVICE dev ,BYTE id,short *getParam)
	
	概要：モータの現在位置を取得する。現在位置の取得は同時に1個のモータしか対応していない

	引数：
		HID_UART_DEVICE dev	… 通信ハンドル
		BYTE id				… 現在位置を取得するモータのID
		short *getParam		… 取得した現在位置を格納する変数へのポインタ

	戻り値：
		正常に処理を実行できたらTRUE、そうでなければFALSE

*************************************************/
int RSGetAngle( HID_UART_DEVICE dev ,BYTE id,short *getParam )
{
	unsigned char	sendbuf[32];
	unsigned char	readbuf[128];
	unsigned char	sum;
	DWORD			i;
	int				ret;
	unsigned long	len, readlen;
	short			angle;

	// バッファクリア
	memset( sendbuf, 0x00, sizeof( sendbuf ));

	// パケット作成
	sendbuf[0]  = (unsigned char)0xFA;				// ヘッダー1
	sendbuf[1]  = (unsigned char)0xAF;				// ヘッダー2
	sendbuf[2]  = (unsigned char)id;				// サーボID
	sendbuf[3]  = (unsigned char)0x0f;				// フラグ(0x0f=指定アドレスから指定バイト取得)
	sendbuf[4]  = (unsigned char)0x2a;				// 取得すつデータのアドレス(現在位置=0x2a)
	sendbuf[5]  = (unsigned char)0x02;				// 取得するデータの長さ(現在位置=2byte)
	sendbuf[6]  = (unsigned char)0x00;				// 個数(0)

	
	// チェックサムの計算
	sum = sendbuf[2];
	for( i = 3; i < 7; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[7] = sum;								// チェックサム

	// パケットを送信
	ret = HidUart_Write(dev,sendbuf, 8, &len);

	//ローカルエコーを読み取り
	if(!ReadLocalEcho(dev,sendbuf,len)) return FALSE;

	// もし送信できたパケット長がデータサイズよりも小さい場合、エラー
	if( len < 8 ) return FALSE;


	// 受信バッファの読み込み
	memset( readbuf, 0x00, sizeof( readbuf ));

	//受信バッファのパケット長の計算
	//	Header(2byte) + ID(1byte) + Flags(1byte) + Address(1byte) + Length(1byte) + Count(1byte) + Dada(2byte) + Sum(1byte)
	readlen = (2) + (1) + (1) + (1) + (1) + (1) + (2) + (1);
	len = 0;

	//バッファの受信
	HidUart_Read(dev,readbuf, readlen, &len);

	//受信バッファのパケット長が、計算で求めた長さ(readlen)と異なる場合、エラー
	if( len < readlen)  return FALSE;

	// 受信データのチェックサム確認
	sum = readbuf[2];
	for( i = 3; i < readlen; i++ ){
		sum = sum ^ readbuf[i];
	}

	//チェックサムが異なる場合、エラー
	if( sum ) return FALSE;

	//受信バッファから、読み取った現在位置のデータを取り出す
	angle = ((readbuf[8] << 8) & 0x0000FF00) | (readbuf[7] & 0x000000FF);
	if(getParam) *getParam = angle;

#ifdef	NEWDEF
	if(id==1 || id==3) *getParam *= -1;
#endif

	return TRUE;
}


/************************************************

	int RSMove( HID_UART_DEVICE dev , short *sPoss, unsigned short sTime ,BYTE id,int num)
	
	概要：モータを指定の角度に動かす。IDが連続した複数のモータを一度に動かすことが可能。

	引数：
		HID_UART_DEVICE dev	… 通信ハンドル
		short *sPoss		… 目標位置を記録した配列変数へのポインタ。開始IDの目標値は、バッファの先頭から順に使用される
		unsigned short sTime … 目標位置までの遷移時間(10ミリ秒単位)
		BYTE id				… 動かすモータのID（複数のモータを動かす場合、先頭のモータのID）
		int num				… 動かすモータ数

	戻り値：
		正常に処理を実行できたらTRUE、そうでなければFALSE

*************************************************/
int RSMove( HID_UART_DEVICE dev , short *sPoss, unsigned short sTime ,BYTE id,int num )
{
	unsigned char	sendbuf[256],*bufp;
	unsigned char	sum;
	unsigned char	i;
	int				ret;
	unsigned long	len,data_len;


	// バッファクリア
	memset( sendbuf, 0x00, sizeof( sendbuf ));

	// パケット作成
	sendbuf[0]  = (unsigned char)0xFA;				    // ヘッダー1
	sendbuf[1]  = (unsigned char)0xAF;				    // ヘッダー2
	sendbuf[2]  = (unsigned char)0;						// ID(0)
	sendbuf[3]  = (unsigned char)0x00;				    // フラグ(0x00)
	sendbuf[4]  = (unsigned char)0x1E;				    // アドレス(0x1E=30)
	sendbuf[5]  = (unsigned char)0x04+1;			    // 長さ(4byte)
	sendbuf[6]  = (unsigned char)num;				    // モータの個数

	//共通部分のパケット長を記録
	data_len = 7;

	//個別のデータ作成
	bufp = &sendbuf[7];
	for(i=0;i<num;i++){
		*bufp++ = id+i;								//モータID
		data_len++;									//パケット長を1byte加算

#ifdef	NEWDEF
		if(id+i==1 || id+i==3) sPoss[i] *= -1;
#endif

		//目標位置をバッファに書き込み(2byte)
		*bufp++ = (unsigned char)(sPoss[i]&0x00FF);
		*bufp++ = (unsigned char)((sPoss[i]&0xFF00)>>8);
		data_len+=2;								//パケット長を2byte加算

		//遷移時間をバッファに書き込み(2byte)
		*bufp++ = (unsigned char)(sTime&0x00FF);
		*bufp++ = (unsigned char)((sTime&0xFF00)>>8);
		data_len+=2;								//パケット長を2byte加算
	}


	// チェックサムの計算
	sum = sendbuf[2];
	for( i = 3; i < data_len; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[data_len] = sum;						// 送信バッファにチェックサムを追加
	data_len++;										//パケット長を1byte加算

	// パケットを送信
	ret = HidUart_Write( dev, (BYTE*) sendbuf, data_len, &len );
	if(ret!=HID_UART_SUCCESS) return FALSE;

	//ローカルエコーの読み取り
	return ReadLocalEcho(dev,sendbuf,data_len);

}


/************************************************

	int ReadLocalEcho(HID_UART_DEVICE dev ,unsigned char *sendbuf,DWORD data_len)
	
	概要：
		送信メッセージのローカルエコーを読み出す関数。
		引数 data_len の長さだけメッセージを受信し、 引数 *sendbuf の内容と同一か比較を行う

	引数：
		HID_UART_DEVICE dev		… 通信ハンドル
		unsigned char *sendbuf	… 送信メッセージ内容へのポインタ
		DWORD data_len			… 送信メッセージのサイズ

	戻り値：
		送信メッセージと同一の内容を受信できたらTRUE、そうでなければFALSE

*************************************************/
int ReadLocalEcho( HID_UART_DEVICE dev ,unsigned char *sendbuf,DWORD data_len )
{

	unsigned char readbuf[1024];
	DWORD len=0;
	memset(readbuf,0,sizeof(readbuf));

	//data_len のサイズだけメッセージを受信
	HidUart_Read( dev, (BYTE*) readbuf, data_len, &len );

	//受信メッセージのサイズが異なる場合、エラー
	if(data_len!=len) return FALSE;

	//受信メッセージと送信メッセージを比較
	for(DWORD i=0;i<len;i++){

		//受信メッセージと送信メッセージが異なる場合、エラー
		if(readbuf[i]!=sendbuf[i]) return FALSE;
	}
	return TRUE;
}

/************************************************

	int CartesianLimitJudgement( double x , double y , double z )
	
	概要：
		Catesianソフトリミットの判定を行う

	引数：
		double x	… x[mm]
		double y	… y[mm]
		double z	… z[mm]

	戻り値：
		Cartesianソフトリミットを満たしていればTRUE、そうでなければFALSE

*************************************************/
int CartesianLimitJudgement( double x , double y , double z ){
	if(( x > CatesianLimit.x.Upper )||
		( x < CatesianLimit.x.Lower )||
		( y > CatesianLimit.y.Upper )||
		( y < CatesianLimit.y.Lower )||
		( z > CatesianLimit.z.Upper )||
		( z < CatesianLimit.z.Lower )
		){
		return FALSE;
	}

	return TRUE;
}

/************************************************

	int JointLimitJudgement( short *JointAngle )
	
	概要：
		Jointソフトリミットの判定を行う

	引数：
		short *JointAngle	…	モータ指令値

	戻り値：
		Jointソフトリミットを満たしていればTRUE、そうでなければFALSE

*************************************************/
int JointLimitJudgement( short *JointAngle ){
	
	for(int i=0;i<servoNum;i++){
		if(( JointAngle[i] > JointLimit[i].Upper )||
			( JointAngle[i] < JointLimit[i].Lower )
			){
			return FALSE;
		}
	}
	
	return TRUE;
}

