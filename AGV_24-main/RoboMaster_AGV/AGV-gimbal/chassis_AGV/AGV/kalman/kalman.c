
#include "kalman.h"

/**
  * @name   kalmanCreate
  * @brief  创建一个卡尔曼滤波器
  * @param  p:  滤波器
  *         T_Q:系统噪声协方差  //Q无限代表只用测量值
  *         T_R:测量噪声协方差
  *         
  * @retval none
  */
  // R固定，Q越大，代表越信任侧量值，Q无穷代表只用测量值
  //		       	反之，Q越小代表越信任模型预测值，Q为零则是只用模型预测
void kalmanCreate(kalman *p,float T_Q,float T_R)  
{
    //kalman* p = ( kalman*)malloc(sizeof( kalman));
    p->X_last = (float)0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
    p->H = 1;
    p->X_mid = p->X_last;
    //return p;
}

/**
  * @name   KalmanFilter
  * @brief  卡尔曼滤波器
  * @param  p:  滤波器
  *         dat:待滤波数据
  * @retval 滤波后的数据
  */

float KalmanFilter(kalman* p,float dat)
{
    p->X_mid =p->A*p->X_last;                     //x(k|k-1) = AX(k-1|k-1)+BU(k)
    p->P_mid = p->A*p->P_last+p->Q;               //p(k|k-1) = Ap(k-1|k-1)A'+Q
    p->kg = p->P_mid/(p->P_mid+p->R);             //kg(k) = p(k|k-1)H'/(Hp(k|k-1)'+R)
    p->X_now = p->X_mid+p->kg*(dat-p->X_mid);     //x(k|k) = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
    p->P_now = (1-p->kg)*p->P_mid;                //p(k|k) = (I-kg(k)H)P(k|k-1)
    p->P_last = p->P_now;                         //状态更新
    p->X_last = p->X_now;
    return p->X_now;
}


/**    
  * 二阶卡尔曼滤波器                                                                       
  */

float matrix_value1;
float matrix_value2;

void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
{
  mat_init(&F->xhat, 2, 1, (float *)I->xhat_data);
  mat_init(&F->xhatminus, 2, 1, (float *)I->xhatminus_data);
  mat_init(&F->z, 2, 1, (float *)I->z_data);
  mat_init(&F->A, 2, 2, (float *)I->A_data);
  mat_init(&F->H, 2, 2, (float *)I->H_data);
  mat_init(&F->Q, 2, 2, (float *)I->Q_data);
  mat_init(&F->R, 2, 2, (float *)I->R_data);
  mat_init(&F->P, 2, 2, (float *)I->P_data);
  mat_init(&F->Pminus, 2, 2, (float *)I->Pminus_data);
  mat_init(&F->K, 2, 2, (float *)I->K_data);
  mat_init(&F->AT, 2, 2, (float *)I->AT_data);
  mat_trans(&F->A, &F->AT);
  mat_init(&F->HT, 2, 2, (float *)I->HT_data);
  mat_trans(&F->H, &F->HT);
  //  matrix_value2 = F->A.pData[1];
}

// xhatminus==x(k|k-1)  xhat==X(k-1|k-1)
// Pminus==p(k|k-1)     P==p(k-1|k-1)    AT==A'
// HT==H'   K==kg(k)    I=1
//

/**
  *@param 卡尔曼参数结构体
  *@param 角度
  *@param 速度
*/
float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2)
{
  float TEMP_data[4] = {0, 0, 0, 0};
  float TEMP_data21[2] = {0, 0};
  mat TEMP, TEMP21;

  mat_init(&TEMP, 2, 2, (float *)TEMP_data);     //
  mat_init(&TEMP21, 2, 1, (float *)TEMP_data21); //

  F->z.pData[0] = signal1; //z(k)
  F->z.pData[1] = signal2; //z(k)

  //1. xhat'(k)= A xhat(k-1)
  mat_mult(&F->A, &F->xhat, &F->xhatminus); //  x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)

  //2. P'(k) = A P(k-1) AT + Q
  mat_mult(&F->A, &F->P, &F->Pminus);  //   p(k|k-1) = A*p(k-1|k-1)*A'+Q
  mat_mult(&F->Pminus, &F->AT, &TEMP); //  p(k|k-1) = A*p(k-1|k-1)*A'+Q
  mat_add(&TEMP, &F->Q, &F->Pminus);   //    p(k|k-1) = A*p(k-1|k-1)*A'+Q

  //3. K(k) = P'(k) HT / (H P'(k) HT + R)
  mat_mult(&F->H, &F->Pminus, &F->K); //  kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
  mat_mult(&F->K, &F->HT, &TEMP);     //      kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
  mat_add(&TEMP, &F->R, &F->K);       //        kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)

  mat_inv(&F->K, &F->P);               //
  mat_mult(&F->Pminus, &F->HT, &TEMP); //
  mat_mult(&TEMP, &F->P, &F->K);       //

  //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
  mat_mult(&F->H, &F->xhatminus, &TEMP21);   //      x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_sub(&F->z, &TEMP21, &F->xhat);         //            x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_mult(&F->K, &F->xhat, &TEMP21);        //           x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_add(&F->xhatminus, &TEMP21, &F->xhat); //    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))

  //5. P(k) = (1-K(k)H)P'(k)
  mat_mult(&F->K, &F->H, &F->P); //            p(k|k) = (I-kg(k)*H)*P(k|k-1)
  mat_sub(&F->Q, &F->P, &TEMP);  //
  mat_mult(&TEMP, &F->Pminus, &F->P);

  matrix_value1 = F->xhat.pData[0];
  matrix_value2 = F->xhat.pData[1];

  F->filtered_value[0] = F->xhat.pData[0];
  F->filtered_value[1] = F->xhat.pData[1];
  return F->filtered_value;
}
