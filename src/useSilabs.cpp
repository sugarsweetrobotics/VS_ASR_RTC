#include "useSilabs.h"


int hr;//�Ԃ�l
HID_UART_DEVICE dev;	//�ʐM�n���h��
short MotorPosition[5] = {0,0,0,0,0};//���[�^�l�ő��
short StartMotorPosition[5] = {0,0,0,0,0};//���[�^�l�ő��
short HomeMotorPosition[5] = {0,0,0,0,0};//���_���A�ʒu
int servoNum = 5;//�T�[�{��
int CartesianSpeed = 0;//�������W��Ԃł̑��x[%]
int JointSpeed = 0;//�֐ߍ��W��Ԃł̑��x[%]

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
	
	�T�v�F���[�^�̃g���N��؂�ւ���֐��BID���A�����������̃��[�^�𓯎��ɐݒ�\�B

	�����F
		HID_UART_DEVICE dev	�c �ʐM�n���h��
		short sMode			�c �Q�C����ON/OFF���w��B0=off�A1=ON
		BYTE id				�c �؂�ւ����J�n���郂�[�^��ID�i�����̃��[�^��؂�ւ���ꍇ�A�擪�̃��[�^��ID�j
		int num				�c �؂�ւ����s�����[�^��

	�߂�l�F
		����ɏ��������s�ł�����TRUE�A�����łȂ����FALSE

*************************************************/
int RSTorqueOnOff( HID_UART_DEVICE dev, short sMode ,BYTE id,int num )
{
	unsigned char	sendbuf[256],*bufp;				//���M�o�b�t�@�֌W
	unsigned char	sum;							//�`�F�b�N�T���v�Z�p
	int				ret;							//�߂�l�L�^�p
	DWORD			data_len=0,len=0;				//�p�P�b�g���Ə������݃T�C�Y�擾�p
	unsigned char	i;


	//���M�o�b�t�@�N���A
	memset( sendbuf, 0x00, sizeof( sendbuf ));

	//�p�P�b�g�쐬
	//1�D�w�b�_�E���ʕ����̍쐬
	sendbuf[0]  = (unsigned char)0xFA;				// �w�b�_�[1
	sendbuf[1]  = (unsigned char)0xAF;				// �w�b�_�[2
	sendbuf[2]  = (unsigned char)0x00;				// �T�[�{ID(���0)
	sendbuf[3]  = (unsigned char)0x00;				// �t���O(���0)
	sendbuf[4]  = (unsigned char)0x24;				// �A�h���X(�g���NON/OFF 0x24=36)
	sendbuf[5]  = (unsigned char)0x01+1;			// ����(1byte)
	sendbuf[6]  = (unsigned char)num;				// ���[�^�̌�

	//���ʕ����̃p�P�b�g�����L�^
	data_len = 7;			

	//2�D�T�[�{�ʕ����̃p�P�b�g�쐬
	bufp = &sendbuf[7];								// ���M�o�b�t�@�̌ʃ��b�Z�[�W�����̊J�n�A�h���X����

	//���������郂�[�^�̌��������A�ʂ̃p�P�b�g��ǉ�
	for(i=0;i<num;i++){
		*bufp++ = id+i;								//���[�^��ID
		data_len++;									//�p�P�b�g����1byte���Z

		*bufp++ = (unsigned char)(sMode&0x00FF);	//ON�EOFF�t���O
		data_len++;									//�p�P�b�g����1byte���Z
	}

	//3�D�`�F�b�N�T���̌v�Z
	//�`�F�b�N�T���́A���M�o�b�t�@��3byte��(�T�[�{ID)�`�I�[��1byte����XOR�����l�ł��B
	sum = sendbuf[2];
	for( i = 3; i < data_len; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[data_len] = sum;						// ���܂����`�F�b�N�T���𑗐M�o�b�t�@�̍Ō�ɑ��
	data_len++;										//�p�P�b�g����1byte���Z

	//4�D���b�Z�[�W�̑��M
	ret = HidUart_Write( dev, (BYTE*) sendbuf, data_len, &len );

	if(ret!=HID_UART_SUCCESS) return FALSE;

	//5�D���[�J���G�R�[�̓ǂݎ��
	return ReadLocalEcho(dev,sendbuf,data_len);
}



/************************************************

	int RSGetAngle( HID_UART_DEVICE dev ,BYTE id,short *getParam)
	
	�T�v�F���[�^�̌��݈ʒu���擾����B���݈ʒu�̎擾�͓�����1�̃��[�^�����Ή����Ă��Ȃ�

	�����F
		HID_UART_DEVICE dev	�c �ʐM�n���h��
		BYTE id				�c ���݈ʒu���擾���郂�[�^��ID
		short *getParam		�c �擾�������݈ʒu���i�[����ϐ��ւ̃|�C���^

	�߂�l�F
		����ɏ��������s�ł�����TRUE�A�����łȂ����FALSE

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

	// �o�b�t�@�N���A
	memset( sendbuf, 0x00, sizeof( sendbuf ));

	// �p�P�b�g�쐬
	sendbuf[0]  = (unsigned char)0xFA;				// �w�b�_�[1
	sendbuf[1]  = (unsigned char)0xAF;				// �w�b�_�[2
	sendbuf[2]  = (unsigned char)id;				// �T�[�{ID
	sendbuf[3]  = (unsigned char)0x0f;				// �t���O(0x0f=�w��A�h���X����w��o�C�g�擾)
	sendbuf[4]  = (unsigned char)0x2a;				// �擾���f�[�^�̃A�h���X(���݈ʒu=0x2a)
	sendbuf[5]  = (unsigned char)0x02;				// �擾����f�[�^�̒���(���݈ʒu=2byte)
	sendbuf[6]  = (unsigned char)0x00;				// ��(0)

	
	// �`�F�b�N�T���̌v�Z
	sum = sendbuf[2];
	for( i = 3; i < 7; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[7] = sum;								// �`�F�b�N�T��

	// �p�P�b�g�𑗐M
	ret = HidUart_Write(dev,sendbuf, 8, &len);

	//���[�J���G�R�[��ǂݎ��
	if(!ReadLocalEcho(dev,sendbuf,len)) return FALSE;

	// �������M�ł����p�P�b�g�����f�[�^�T�C�Y�����������ꍇ�A�G���[
	if( len < 8 ) return FALSE;


	// ��M�o�b�t�@�̓ǂݍ���
	memset( readbuf, 0x00, sizeof( readbuf ));

	//��M�o�b�t�@�̃p�P�b�g���̌v�Z
	//	Header(2byte) + ID(1byte) + Flags(1byte) + Address(1byte) + Length(1byte) + Count(1byte) + Dada(2byte) + Sum(1byte)
	readlen = (2) + (1) + (1) + (1) + (1) + (1) + (2) + (1);
	len = 0;

	//�o�b�t�@�̎�M
	HidUart_Read(dev,readbuf, readlen, &len);

	//��M�o�b�t�@�̃p�P�b�g�����A�v�Z�ŋ��߂�����(readlen)�ƈقȂ�ꍇ�A�G���[
	if( len < readlen)  return FALSE;

	// ��M�f�[�^�̃`�F�b�N�T���m�F
	sum = readbuf[2];
	for( i = 3; i < readlen; i++ ){
		sum = sum ^ readbuf[i];
	}

	//�`�F�b�N�T�����قȂ�ꍇ�A�G���[
	if( sum ) return FALSE;

	//��M�o�b�t�@����A�ǂݎ�������݈ʒu�̃f�[�^�����o��
	angle = ((readbuf[8] << 8) & 0x0000FF00) | (readbuf[7] & 0x000000FF);
	if(getParam) *getParam = angle;

#ifdef	NEWDEF
	if(id==1 || id==3) *getParam *= -1;
#endif

	return TRUE;
}


/************************************************

	int RSMove( HID_UART_DEVICE dev , short *sPoss, unsigned short sTime ,BYTE id,int num)
	
	�T�v�F���[�^���w��̊p�x�ɓ������BID���A�����������̃��[�^����x�ɓ��������Ƃ��\�B

	�����F
		HID_UART_DEVICE dev	�c �ʐM�n���h��
		short *sPoss		�c �ڕW�ʒu���L�^�����z��ϐ��ւ̃|�C���^�B�J�nID�̖ڕW�l�́A�o�b�t�@�̐擪���珇�Ɏg�p�����
		unsigned short sTime �c �ڕW�ʒu�܂ł̑J�ڎ���(10�~���b�P��)
		BYTE id				�c ���������[�^��ID�i�����̃��[�^�𓮂����ꍇ�A�擪�̃��[�^��ID�j
		int num				�c ���������[�^��

	�߂�l�F
		����ɏ��������s�ł�����TRUE�A�����łȂ����FALSE

*************************************************/
int RSMove( HID_UART_DEVICE dev , short *sPoss, unsigned short sTime ,BYTE id,int num )
{
	unsigned char	sendbuf[256],*bufp;
	unsigned char	sum;
	unsigned char	i;
	int				ret;
	unsigned long	len,data_len;


	// �o�b�t�@�N���A
	memset( sendbuf, 0x00, sizeof( sendbuf ));

	// �p�P�b�g�쐬
	sendbuf[0]  = (unsigned char)0xFA;				    // �w�b�_�[1
	sendbuf[1]  = (unsigned char)0xAF;				    // �w�b�_�[2
	sendbuf[2]  = (unsigned char)0;						// ID(0)
	sendbuf[3]  = (unsigned char)0x00;				    // �t���O(0x00)
	sendbuf[4]  = (unsigned char)0x1E;				    // �A�h���X(0x1E=30)
	sendbuf[5]  = (unsigned char)0x04+1;			    // ����(4byte)
	sendbuf[6]  = (unsigned char)num;				    // ���[�^�̌�

	//���ʕ����̃p�P�b�g�����L�^
	data_len = 7;

	//�ʂ̃f�[�^�쐬
	bufp = &sendbuf[7];
	for(i=0;i<num;i++){
		*bufp++ = id+i;								//���[�^ID
		data_len++;									//�p�P�b�g����1byte���Z

#ifdef	NEWDEF
		if(id+i==1 || id+i==3) sPoss[i] *= -1;
#endif

		//�ڕW�ʒu���o�b�t�@�ɏ�������(2byte)
		*bufp++ = (unsigned char)(sPoss[i]&0x00FF);
		*bufp++ = (unsigned char)((sPoss[i]&0xFF00)>>8);
		data_len+=2;								//�p�P�b�g����2byte���Z

		//�J�ڎ��Ԃ��o�b�t�@�ɏ�������(2byte)
		*bufp++ = (unsigned char)(sTime&0x00FF);
		*bufp++ = (unsigned char)((sTime&0xFF00)>>8);
		data_len+=2;								//�p�P�b�g����2byte���Z
	}


	// �`�F�b�N�T���̌v�Z
	sum = sendbuf[2];
	for( i = 3; i < data_len; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[data_len] = sum;						// ���M�o�b�t�@�Ƀ`�F�b�N�T����ǉ�
	data_len++;										//�p�P�b�g����1byte���Z

	// �p�P�b�g�𑗐M
	ret = HidUart_Write( dev, (BYTE*) sendbuf, data_len, &len );
	if(ret!=HID_UART_SUCCESS) return FALSE;

	//���[�J���G�R�[�̓ǂݎ��
	return ReadLocalEcho(dev,sendbuf,data_len);

}


/************************************************

	int ReadLocalEcho(HID_UART_DEVICE dev ,unsigned char *sendbuf,DWORD data_len)
	
	�T�v�F
		���M���b�Z�[�W�̃��[�J���G�R�[��ǂݏo���֐��B
		���� data_len �̒����������b�Z�[�W����M���A ���� *sendbuf �̓��e�Ɠ��ꂩ��r���s��

	�����F
		HID_UART_DEVICE dev		�c �ʐM�n���h��
		unsigned char *sendbuf	�c ���M���b�Z�[�W���e�ւ̃|�C���^
		DWORD data_len			�c ���M���b�Z�[�W�̃T�C�Y

	�߂�l�F
		���M���b�Z�[�W�Ɠ���̓��e����M�ł�����TRUE�A�����łȂ����FALSE

*************************************************/
int ReadLocalEcho( HID_UART_DEVICE dev ,unsigned char *sendbuf,DWORD data_len )
{

	unsigned char readbuf[1024];
	DWORD len=0;
	memset(readbuf,0,sizeof(readbuf));

	//data_len �̃T�C�Y�������b�Z�[�W����M
	HidUart_Read( dev, (BYTE*) readbuf, data_len, &len );

	//��M���b�Z�[�W�̃T�C�Y���قȂ�ꍇ�A�G���[
	if(data_len!=len) return FALSE;

	//��M���b�Z�[�W�Ƒ��M���b�Z�[�W���r
	for(DWORD i=0;i<len;i++){

		//��M���b�Z�[�W�Ƒ��M���b�Z�[�W���قȂ�ꍇ�A�G���[
		if(readbuf[i]!=sendbuf[i]) return FALSE;
	}
	return TRUE;
}

/************************************************

	int CartesianLimitJudgement( double x , double y , double z )
	
	�T�v�F
		Catesian�\�t�g���~�b�g�̔�����s��

	�����F
		double x	�c x[mm]
		double y	�c y[mm]
		double z	�c z[mm]

	�߂�l�F
		Cartesian�\�t�g���~�b�g�𖞂����Ă����TRUE�A�����łȂ����FALSE

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
	
	�T�v�F
		Joint�\�t�g���~�b�g�̔�����s��

	�����F
		short *JointAngle	�c	���[�^�w�ߒl

	�߂�l�F
		Joint�\�t�g���~�b�g�𖞂����Ă����TRUE�A�����łȂ����FALSE

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

