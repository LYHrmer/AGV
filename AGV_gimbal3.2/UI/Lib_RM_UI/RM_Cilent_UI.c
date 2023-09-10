/*************************************************************

RM�Զ���UIЭ��       ����RM2020ѧ������ͨ��Э��V1.1

ɽ������ѧ ����ս�� ����@Rjgawuie

**************************************************************/
#include "RM_Cilent_UI.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"
#include "crc8_crc16.h"
#include "math.h"
#include "cmsis_os.h"
#include "referee.h"
unsigned char UI_Seq;                      //�����
/****************************************��������ӳ��************************************/
void UI_SendByte(unsigned char ch)
{
	HAL_UART_Transmit(&huart6, (uint8_t*)&ch, 1,999);
}

/********************************************ɾ������*************************************
**������Del_Operate  ��Ӧͷ�ļ�ɾ������
        Del_Layer    Ҫɾ���Ĳ� ȡֵ0-9
*****************************************************************************************/

void UI_Delete(u8 Del_Operate,u8 Del_Layer)
{

  
  unsigned char *framepoint;                      //��дָ��
	u16 frametail=0xFFFF;                        //CRC16У��ֵ
	int loop_control;                       //For����ѭ������

	UI_Packhead framehead;
	UI_Data_Operate datahead;
	UI_Data_Delete del;

	framepoint=(unsigned char *)&framehead;

	framehead.SOF=UI_SOF;
	framehead.Data_Length=8;
	framehead.Seq=UI_Seq;
	framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
	framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //����ͷ����

	datahead.Data_ID=UI_Data_ID_Del;
	 /*SZL 6-16-2022 ��̬����ȡ ����ϵͳID*/
	datahead.Sender_ID=get_robot_id(); //Robot_ID;
	switch(get_robot_id()){
		case UI_Data_RobotID_RHero:
			datahead.Receiver_ID = UI_Data_CilentID_RHero;//ΪӢ�۲����ֿͻ���(��)
			break;
		case UI_Data_RobotID_REngineer:
			datahead.Receiver_ID = UI_Data_CilentID_REngineer;
			break;
		case UI_Data_RobotID_RStandard1:
			datahead.Receiver_ID = UI_Data_CilentID_RStandard1;
			break;
		case UI_Data_RobotID_RStandard2:
			datahead.Receiver_ID = UI_Data_CilentID_RStandard2;
			break;
		case UI_Data_RobotID_RStandard3:
			datahead.Receiver_ID = UI_Data_CilentID_RStandard3;
			break;
		case UI_Data_RobotID_RAerial:
			datahead.Receiver_ID = UI_Data_CilentID_RAerial;
			break;

		case UI_Data_RobotID_BHero:
			datahead.Receiver_ID = UI_Data_CilentID_BHero;
			break;
		case UI_Data_RobotID_BEngineer:
			datahead.Receiver_ID = UI_Data_CilentID_BEngineer;
			break;
		case UI_Data_RobotID_BStandard1:
			datahead.Receiver_ID = UI_Data_CilentID_BStandard1;
			break;
		case UI_Data_RobotID_BStandard2:
			datahead.Receiver_ID = UI_Data_CilentID_BStandard2;
			break;
		case UI_Data_RobotID_BStandard3:
			datahead.Receiver_ID = UI_Data_CilentID_BStandard3;
			break;
		case UI_Data_RobotID_BAerial:
			datahead.Receiver_ID = UI_Data_CilentID_BAerial;
			break;
		default :
			datahead.Receiver_ID = Cilent_ID; //Ĭ�� ����������һ����ȥ
			datahead.Sender_ID = Robot_ID;
			break;
	 }//����������
   
	del.Delete_Operate=Del_Operate;
	del.Layer=Del_Layer;                                     //������Ϣ

	frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
	framepoint=(unsigned char *)&datahead;
	frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);
	framepoint=(unsigned char *)&del;
	frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(del),frametail);  //CRC16У��ֵ����

	framepoint=(unsigned char *)&framehead;
	for(loop_control=0;loop_control<sizeof(framehead);loop_control++)
	{
	  UI_SendByte(*framepoint);
	  framepoint++;
	}
	framepoint=(unsigned char *)&datahead;
	for(loop_control=0;loop_control<sizeof(datahead);loop_control++)
	{
	  UI_SendByte(*framepoint);
	  framepoint++;
	}
	framepoint=(unsigned char *)&del;
	for(loop_control=0;loop_control<sizeof(del);loop_control++)
	{
	  UI_SendByte(*framepoint);
	  framepoint++;
	}                                                                 //��������֡
	framepoint=(unsigned char *)&frametail;
	for(loop_control=0;loop_control<sizeof(frametail);loop_control++)
	{
	  UI_SendByte(*framepoint);
	  framepoint++;                                                  //����CRC16У��ֵ
	}

	UI_Seq++;                                                         //�����+1
}
/************************************************����ֱ��*************************************************
**������*image Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
        imagename[3]   ͼƬ���ƣ����ڱ�ʶ����
        Graph_Operate   ͼƬ��������ͷ�ļ�
        Graph_Layer    ͼ��0-9
        Graph_Color    ͼ����ɫ
        Graph_Width    ͼ���߿�
        Start_x��Start_x    ��ʼ����
        End_x��End_y   ��������
**********************************************************************************************************/
        
        
void Line_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 End_x,u32 End_y)
{
   int i;
   for(i=0;i<3&&imagename[i]!='\0';i++)
      image->graphic_name[2-i]=imagename[i];
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->end_x = End_x;
   image->end_y = End_y;
}

/************************************************���ƾ���*************************************************
**������*image Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
        imagename[3]   ͼƬ���ƣ����ڱ�ʶ����
        Graph_Operate   ͼƬ��������ͷ�ļ�
        Graph_Layer    ͼ��0-9
        Graph_Color    ͼ����ɫ
        Graph_Width    ͼ���߿�
        Start_x��Start_x    ��ʼ����
        End_x��End_y   �������꣨�Զ������꣩
**********************************************************************************************************/
        
void Rectangle_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 End_x,u32 End_y)
{
   int i;
   for(i=0;i<3&&imagename[i]!='\0';i++)
      image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Rectangle;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->end_x = End_x;
   image->end_y = End_y;
}

/************************************************������Բ*************************************************
**������*image Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
        imagename[3]   ͼƬ���ƣ����ڱ�ʶ����
        Graph_Operate   ͼƬ��������ͷ�ļ�
        Graph_Layer    ͼ��0-9
        Graph_Color    ͼ����ɫ
        Graph_Width    ͼ���߿�
        Start_x��Start_x    Բ������
        Graph_Radius  ͼ�ΰ뾶
**********************************************************************************************************/
        
        
void Circle_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 Graph_Radius)
{
   int i;
   for(i=0;i<3&&imagename[i]!='\0';i++)
      image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Circle;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->radius = Graph_Radius;
}

/************************************************����Բ��*************************************************
**������*image Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
        imagename[3]   ͼƬ���ƣ����ڱ�ʶ����
        Graph_Operate   ͼƬ��������ͷ�ļ�
        Graph_Layer    ͼ��0-9
        Graph_Color    ͼ����ɫ
        Graph_Width    ͼ���߿�
        Graph_StartAngle,Graph_EndAngle    ��ʼ����ֹ�Ƕ�
        Start_y,Start_y    Բ������
        x_Length,y_Length   x,y�������᳤���ο���Բ
**********************************************************************************************************/
        
void Arc_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_StartAngle,u32 Graph_EndAngle,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 x_Length,u32 y_Length)
{
   int i;
   
   for(i=0;i<3&&imagename[i]!='\0';i++)
      image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Arc;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->start_angle = Graph_StartAngle;
   image->end_angle = Graph_EndAngle;
   image->end_x = x_Length;
   image->end_y = y_Length;
}




/************************************************���Ƹ���������*************************************************
**������*image Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
        imagename[3]   ͼƬ���ƣ����ڱ�ʶ����
        Graph_Operate   ͼƬ��������ͷ�ļ�
        Graph_Layer    ͼ��0-9
        Graph_Color    ͼ����ɫ
        Graph_Width    ͼ���߿�
        Graph_Size     �ֺ�
        Graph_Digit    С��λ��
        Start_x��Start_x    ��ʼ����
        Graph_Float   Ҫ��ʾ�ı���
**********************************************************************************************************/
        
void Float_Draw(Float_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Size,u32 Graph_Digit,u32 Graph_Width,u32 Start_x,u32 Start_y,float Graph_Float)
{
   int i;
   
   for(i=0;i<3&&imagename[i]!='\0';i++)
      image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Float;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->start_angle = Graph_Size;
   image->end_angle = Graph_Digit;
   image->graph_Float = Graph_Float;
}


/************************************************�����ַ�������*************************************************
**������*image Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
        imagename[3]   ͼƬ���ƣ����ڱ�ʶ����
        Graph_Operate   ͼƬ��������ͷ�ļ�
        Graph_Layer    ͼ��0-9
        Graph_Color    ͼ����ɫ
        Graph_Width    ͼ���߿�
        Graph_Size     �ֺ�
        Graph_Digit    �ַ�����
        Start_x��Start_x    ��ʼ����
        *Char_Data          �������ַ�����ʼ��ַ
**********************************************************************************************************/
        
void Char_Draw(String_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Size,u32 Graph_Digit,u32 Graph_Width,u32 Start_x,u32 Start_y,char *Char_Data)
{
   int i;
   
   for(i=0;i<3&&imagename[i]!='\0';i++)
      image->Graph_Control.graphic_name[2-i]=imagename[i];
   image->Graph_Control.graphic_tpye = UI_Graph_Char;
   image->Graph_Control.operate_tpye = Graph_Operate;
   image->Graph_Control.layer = Graph_Layer;
   image->Graph_Control.color = Graph_Color;
   image->Graph_Control.width = Graph_Width;
   image->Graph_Control.start_x = Start_x;
   image->Graph_Control.start_y = Start_y;
   image->Graph_Control.start_angle = Graph_Size;
   image->Graph_Control.end_angle = Graph_Digit;
   
   for(i=0;i<Graph_Digit;i++)
   {
      image->show_Data[i]=*Char_Data;
      Char_Data++;
   }
}


/************************************************UI���ͺ�����ʹ������Ч��*********************************
**������ cnt   ͼ�θ���
         ...   ͼ�α�������


Tips�����ú���ֻ������1��2��5��7��ͼ�Σ�������ĿЭ��δ�漰
**********************************************************************************************************/
int UI_ReFresh(int cnt,...)
{
  int i,n;
   Graph_Data imageData;
   unsigned char *framepoint;                      //��дָ��
   u16 frametail=0xFFFF;                        //CRC16У��ֵ
   
   UI_Packhead framehead;//frame_header(5-byte)+cmd_id(2-byte)
   UI_Data_Operate datahead;//�������ݰ�����һ��ͳһ�����ݶ�ͷ�ṹ: ����ID ������ �����ߵ�ID���������ݶ�
   
   va_list ap;
   va_start(ap,cnt);
   
   framepoint=(unsigned char *)&framehead;
   framehead.SOF=UI_SOF;
   framehead.Data_Length=6+cnt*15;
   framehead.Seq=UI_Seq;
   framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
   framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //����ͷ����
   
   switch(cnt)
   {
      case 1:
         datahead.Data_ID=UI_Data_ID_Draw1;
         break;
      case 2:
         datahead.Data_ID=UI_Data_ID_Draw2;
         break;
      case 5:
         datahead.Data_ID=UI_Data_ID_Draw5;
         break;
      case 7:
         datahead.Data_ID=UI_Data_ID_Draw7;
         break;
      default:
         return (-1);
   }
	 
    /*SZL 6-16-2022 ��̬����ȡ ����ϵͳID*/
    datahead.Sender_ID=get_robot_id(); //Robot_ID;
	switch(get_robot_id()){
		case UI_Data_RobotID_RHero:
			datahead.Receiver_ID = UI_Data_CilentID_RHero;//ΪӢ�۲����ֿͻ���(��)
			break;
		case UI_Data_RobotID_REngineer:
			datahead.Receiver_ID = UI_Data_CilentID_REngineer;
			break;
		case UI_Data_RobotID_RStandard1:
			datahead.Receiver_ID = UI_Data_CilentID_RStandard1;
			break;
		case UI_Data_RobotID_RStandard2:
			datahead.Receiver_ID = UI_Data_CilentID_RStandard2;
			break;
		case UI_Data_RobotID_RStandard3:
			datahead.Receiver_ID = UI_Data_CilentID_RStandard3;
			break;
		case UI_Data_RobotID_RAerial:
			datahead.Receiver_ID = UI_Data_CilentID_RAerial;
			break;
		case UI_Data_RobotID_BHero:
			datahead.Receiver_ID = UI_Data_CilentID_BHero;
			break;
		case UI_Data_RobotID_BEngineer:
			datahead.Receiver_ID = UI_Data_CilentID_BEngineer;
			break;
		case UI_Data_RobotID_BStandard1:
			datahead.Receiver_ID = UI_Data_CilentID_BStandard1;
			break;
		case UI_Data_RobotID_BStandard2:
			datahead.Receiver_ID = UI_Data_CilentID_BStandard2;
			break;
		case UI_Data_RobotID_BStandard3:
			datahead.Receiver_ID = UI_Data_CilentID_BStandard3;
			break;
		case UI_Data_RobotID_BAerial:
			datahead.Receiver_ID = UI_Data_CilentID_BAerial;
			break;
		default :
			datahead.Receiver_ID = Cilent_ID; //Ĭ�� ����������һ����ȥ
			datahead.Sender_ID = Robot_ID;
			break;
	}//����������
   
   framepoint=(unsigned char *)&framehead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
   framepoint=(unsigned char *)&datahead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);          //CRC16У��ֵ���㣨���֣�
   
   framepoint=(unsigned char *)&framehead;
   for(i=0;i<sizeof(framehead);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   framepoint=(unsigned char *)&datahead;
   for(i=0;i<sizeof(datahead);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   
   for(i=0;i<cnt;i++)
   {
      imageData=va_arg(ap,Graph_Data);
      
      framepoint=(unsigned char *)&imageData;
      frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(imageData),frametail);             //CRC16У��
      
      for(n=0;n<sizeof(imageData);n++)
      {
         UI_SendByte(*framepoint);
         framepoint++;             
      } //����ͼƬ֡
   }
   framepoint=(unsigned char *)&frametail;
   for(i=0;i<sizeof(frametail);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;                                                  //����CRC16У��ֵ
   }
   
   va_end(ap);
   
   UI_Seq++;                                                         //�����+1
   return 0;
}



/************************************************UI�����ַ���ʹ������Ч��*********************************
**������ string_Data  �ַ�ͼ������
**********************************************************************************************************/
int Char_ReFresh(String_Data string_Data)
{
int i;
   String_Data imageData;
   unsigned char *framepoint;                      //��дָ��
   u16 frametail=0xFFFF;                        //CRC16У��ֵ
   
   UI_Packhead framehead;//frame_header(5-byte)+cmd_id(2-byte)
   UI_Data_Operate datahead;//�������ݰ�����һ��ͳһ�����ݶ�ͷ�ṹ: ����ID ������ �����ߵ�ID���������ݶ�
   imageData=string_Data;
   
   
   framepoint=(unsigned char *)&framehead;
   framehead.SOF=UI_SOF;
   framehead.Data_Length=6+45;
   framehead.Seq=UI_Seq;
   framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
   framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //����ͷ����
   

   datahead.Data_ID=UI_Data_ID_DrawChar;//SZL 6-8-2022 �޸�

    /*SZL 6-16-2022 ��̬����ȡ ����ϵͳID*/
    datahead.Sender_ID=get_robot_id(); //Robot_ID;
	switch(get_robot_id()){
		case UI_Data_RobotID_RHero:
			datahead.Receiver_ID = UI_Data_CilentID_RHero;//ΪӢ�۲����ֿͻ���(��)
			break;
		case UI_Data_RobotID_REngineer:
			datahead.Receiver_ID = UI_Data_CilentID_REngineer;
			break;
		case UI_Data_RobotID_RStandard1:
			datahead.Receiver_ID = UI_Data_CilentID_RStandard1;
			break;
		case UI_Data_RobotID_RStandard2:
			datahead.Receiver_ID = UI_Data_CilentID_RStandard2;
			break;
		case UI_Data_RobotID_RStandard3:
			datahead.Receiver_ID = UI_Data_CilentID_RStandard3;
			break;
		case UI_Data_RobotID_RAerial:
			datahead.Receiver_ID = UI_Data_CilentID_RAerial;
			break;
		case UI_Data_RobotID_BHero:
			datahead.Receiver_ID = UI_Data_CilentID_BHero;
			break;
		case UI_Data_RobotID_BEngineer:
			datahead.Receiver_ID = UI_Data_CilentID_BEngineer;
			break;
		case UI_Data_RobotID_BStandard1:
			datahead.Receiver_ID = UI_Data_CilentID_BStandard1;
			break;
		case UI_Data_RobotID_BStandard2:
			datahead.Receiver_ID = UI_Data_CilentID_BStandard2;
			break;
		case UI_Data_RobotID_BStandard3:
			datahead.Receiver_ID = UI_Data_CilentID_BStandard3;
			break;
		case UI_Data_RobotID_BAerial:
			datahead.Receiver_ID = UI_Data_CilentID_BAerial;
			break;
		default :
			datahead.Receiver_ID = Cilent_ID; //Ĭ�� ����������һ����ȥ
			datahead.Sender_ID = Robot_ID;
			break;
	 }//����������   
   framepoint=(unsigned char *)&framehead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
   framepoint=(unsigned char *)&datahead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);
   framepoint=(unsigned char *)&imageData;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(imageData),frametail);             //CRC16У��   //CRC16У��ֵ���㣨���֣�
   
   framepoint=(unsigned char *)&framehead;
   for(i=0;i<sizeof(framehead);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   framepoint=(unsigned char *)&datahead;
   for(i=0;i<sizeof(datahead);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }                                                   //���Ͳ�������  
   framepoint=(unsigned char *)&imageData;
   for(i=0;i<sizeof(imageData);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;             
   }                                               //����ͼƬ֡
   
   
   
   framepoint=(unsigned char *)&frametail;
   for(i=0;i<sizeof(frametail);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;                                                  //����CRC16У��ֵ
   }
   
   
   UI_Seq++;                                                         //�����+1
   return 0;
}
int Delete_ReFresh(UI_Data_Delete delete_Data)
{
	 int i;
   UI_Data_Delete imageData;
   unsigned char *framepoint;                      //��дָ��
   u16 frametail=0xFFFF;                        //CRC16У��ֵ
   
   UI_Packhead framehead;//frame_header(5-byte)+cmd_id(2-byte)
   UI_Data_Operate datahead;//�������ݰ�����һ��ͳһ�����ݶ�ͷ�ṹ: ����ID ������ �����ߵ�ID���������ݶ�
   imageData=delete_Data;
   
   
   framepoint=(unsigned char *)&framehead;
   framehead.SOF=UI_SOF;
   framehead.Data_Length=6+2;
   framehead.Seq=UI_Seq;
   framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
   framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //����ͷ����
   

   datahead.Data_ID=UI_Data_ID_Del;

    /*SZL 6-16-2022 ��̬����ȡ ����ϵͳID*/
    datahead.Sender_ID=get_robot_id(); //Robot_ID;
	switch(get_robot_id()){
		case UI_Data_RobotID_RHero:
			datahead.Receiver_ID = UI_Data_CilentID_RHero;//ΪӢ�۲����ֿͻ���(��)
			break;
		case UI_Data_RobotID_REngineer:
			datahead.Receiver_ID = UI_Data_CilentID_REngineer;
			break;
		case UI_Data_RobotID_RStandard1:
			datahead.Receiver_ID = UI_Data_CilentID_RStandard1;
			break;
		case UI_Data_RobotID_RStandard2:
			datahead.Receiver_ID = UI_Data_CilentID_RStandard2;
			break;
		case UI_Data_RobotID_RStandard3:
			datahead.Receiver_ID = UI_Data_CilentID_RStandard3;
			break;
		case UI_Data_RobotID_RAerial:
			datahead.Receiver_ID = UI_Data_CilentID_RAerial;
			break;

		case UI_Data_RobotID_BHero:
			datahead.Receiver_ID = UI_Data_CilentID_BHero;
			break;
		case UI_Data_RobotID_BEngineer:
			datahead.Receiver_ID = UI_Data_CilentID_BEngineer;
			break;
		case UI_Data_RobotID_BStandard1:
			datahead.Receiver_ID = UI_Data_CilentID_BStandard1;
			break;
		case UI_Data_RobotID_BStandard2:
			datahead.Receiver_ID = UI_Data_CilentID_BStandard2;
			break;
		case UI_Data_RobotID_BStandard3:
			datahead.Receiver_ID = UI_Data_CilentID_BStandard3;
			break;
		case UI_Data_RobotID_BAerial:
			datahead.Receiver_ID = UI_Data_CilentID_BAerial;
			break;
		default :
			datahead.Receiver_ID = Cilent_ID; //Ĭ�� ����������һ����ȥ
			datahead.Sender_ID = Robot_ID;
			break;
	 }//����������
   
   framepoint=(unsigned char *)&framehead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
   framepoint=(unsigned char *)&datahead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);
   framepoint=(unsigned char *)&imageData;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(imageData),frametail);             //CRC16У��   //CRC16У��ֵ���㣨���֣�
   
   framepoint=(unsigned char *)&framehead;
   for(i=0;i<sizeof(framehead);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   framepoint=(unsigned char *)&datahead;
   for(i=0;i<sizeof(datahead);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }                                                   //���Ͳ�������  
   framepoint=(unsigned char *)&imageData;
   for(i=0;i<sizeof(imageData);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;             
   }                                               //����ͼƬ֡
   

   framepoint=(unsigned char *)&frametail;
   for(i=0;i<sizeof(frametail);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;                                                  //����CRC16У��ֵ
   }
   
   
   UI_Seq++;                                                         //�����+1
   return 0;
}


/*****************************************************CRC8У��ֵ����**********************************************/
const uint8_t CRC8_INIT_UI = 0xff; 
const uint8_t CRC8_TAB_UI[256] = 
{ 
0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41, 
0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 
0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62, 
0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff, 
0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 
0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a, 
0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24, 
0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 
0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd, 
0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50, 
0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 
0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73, 
0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b, 
0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 
0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8, 
0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35, 
};
uint8_t Get_CRC8_Check_Sum_UI(uint8_t *pchMessage,unsigned int dwLength,uint8_t ucCRC8) 
{ 
uint8_t ucIndex; 
while (dwLength--) 
{ 
ucIndex = ucCRC8^(*pchMessage++); 
ucCRC8 = CRC8_TAB_UI[ucIndex]; 
} 
return(ucCRC8); 
}
uint16_t CRC_INIT_UI = 0xffff; 
const uint16_t wCRC_Table_UI[256] = 
{ 
0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 
0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 
0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 
0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 
0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 
0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5, 
0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 
0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974, 
0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb, 
0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 
0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72, 
0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 
0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 
0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738, 
0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 
0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff, 
0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 
0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 
0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5, 
0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 
0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134, 
0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c, 
0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 
0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb, 
0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 
0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 
0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 
0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9, 
0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 
0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};
/* 
** Descriptions: CRC16 checksum function 
** Input: Data to check,Stream length, initialized checksum 
** Output: CRC checksum 
*/ 
uint16_t Get_CRC16_Check_Sum_UI(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC) 
{ 
Uint8_t chData; 
if (pchMessage == NULL) 
{ 
return 0xFFFF; 
} 
while(dwLength--) 
{ 
chData = *pchMessage++;
(wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table_UI[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 
0x00ff]; 
} 
return wCRC; 
}
