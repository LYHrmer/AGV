#include "rudder_task.h"

#define int_abs(x) ((x) > 0 ? (x) : (-x))
#define pi 3.1415926
#define half_pi 1.5707963
#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }                                                    \
//电机编码值规整 0—8191
#define ECD_Format(ecd)         \
    {                           \
        if ((ecd) > ecd_range)  \
            (ecd) -= ecd_range; \
        else if ((ecd) < 0)     \
            (ecd) += ecd_range; \
    }                           
#define error(the,last,out)         \
	{                               \
	   if(the-last<0.0f)out=last-the;\
	  else   out=the-last;          \
		                            \
	}
#define rand(angle)  angle=angle/8191*3.1415926f*2 

#define compare(angle1,angle2,ture) \
   {                                \
      if(angle1>angle2)             \
	  {                             \
         ture=angle2;               \
	   }                            \
      else                          \
      {                             \
	  ture=angle1;                  \
	  }	                            \
   }	                            
 
                              
#define rad(angle) angle=angle/180*3.1415926f  