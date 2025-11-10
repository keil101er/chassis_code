
/****************************RIGHT*******************************/
/****************************CAN1*******************************/
//C板放置：R标横着 R的上半部分对应没有贴纸的，下半部分（竖和娜）娜对应唯一一个贴标签6的电机
//C板CAN线序左L右H

#include <math.h>
#include <stdio.h>
#include "CANdata_analysis.h"
#include "chassisR_task.h"
#include "can.h"
#include "cmsis_os.h"
#include "detect_task.h"
#include "chassis_power_control.h"
#include "shoot.h"
#include "VMC_calc.h"
#define YAW_MOUSE_SEN   0.00005f//0.00005f
#define PITCH_MOUSE_SEN -0.00015f//0.00015f
////reducation of 3508 motor
////m3508电机的减速比
//#define M3508_MOTOR_REDUCATION 15.764705882f

////m3508 rpm change to chassis speed   576.096  3.14 *07425 = 0.233145
////m3508转子转速(rpm)转化成底盘速度(m/s)的比例，c=pi*r/(30*k)，k为电机减速比
//#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN 0.0004998609952f

////m3508 rpm change to motor angular velocity
////m3508转子转速(rpm)转换为输出轴角速度(rad/s)的比例
//#define CHASSIS_MOTOR_RPM_TO_OMG_SEN 0.00664267f

////m3508 current change to motor torque
////m3508转矩电流(-16384~16384)转为成电机输出转矩(N.m)的比例
////c=20/16384*0.3，   
//#define CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN 0.000366211f

//rocker value (max 660) change to vertial speed (m/s) 
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.0100f

//右边
float LQR_K_R[12]={       
-11.0224 , -2.664 ,-11.6436,	-20.2754 , 30.2361 , 3.3863,//      ??????????????????????
10.2554 ,  0.9963 ,   1.0764  ,  3.2532 , 40.8928  , 3.5782	
};
                                                               
//遛弯
//float Poly_Coefficient[12][4] = {{-611.59342, 639.61534, -247.78576, 7.63222},
//	{-123.40484, 125.69906, -50.31432, 2.10250},
//	{-72.27129, 64.81787, -16.29453, -1.47444},
//	{-239.70338, 215.35520, -54.84512, -4.70619},
//	{67.75831, -7.98603, -62.99347, 39.14809},
//	{52.95899, -46.03012, 5.73408, 4.61071},
//	{789.75527, -675.04834, 201.33612, -6.29698}, 
//	{154.84965, -128.67852, 35.23024, -1.53590},
//	{93.54271, -69.08236, 14.07947, 0.33293},
//{314.08769, -232.28542, 47.78625, 0.89134},
//{-58.19551, -36.72993, 67.63760, 44.63026},
//{-73.61757, 49.79165, -4.25481, 2.91098}};

//勉强能用 伸缩腿有抖动
// float Poly_Coefficient[12][4] = {-421.75465, 404.14066, -163.18492, 1.52481, -100.33598, 88.97437, -33.12351, 0.46394, -13.79006, 20.45913, -5.20346, -6.44626, 0.23096, 8.70198, -1.16290, -7.84332, -145.33854, 158.48464, -89.03226, 32.92067, -27.19789, 19.76695, -10.47246, 4.95830, 615.75677, -496.49431, 155.68024, -1.46787, 129.15010, -93.95712, 23.77051, 0.02808, 207.43929, -152.95834, 33.54335, 1.53506, 211.73999, -148.21559, 29.04610, 2.57854, 88.69753, -151.25099, 93.98502, 36.97904, -21.28958, 10.81717, 4.85406, 2.35990};

//招新
// float Poly_Coefficient[12][4] = {-750.37091, 674.65333, -264.83487, 6.61073, -165.70860, 130.33069, -48.34246, 1.99983, -112.41372, 88.27984, -22.36833, -1.81057, -274.12904, 213.96496, -54.59379, -4.26855, 124.07351, 6.68282, -83.93672, 45.00612, 84.42312, -52.97705, 3.26273, 6.17020, 1002.54935, -908.03209, 299.65431, -1.61756, 172.43066, -143.93387, 38.95117, -0.98294, 95.29996, -61.97671, 6.31565, 2.44423, 236.65063, -155.63029, 17.58057, 5.32039, 331.06497, -457.91684, 244.40584, 32.70065, -35.98225, -2.76939, 21.70274, 2.72641};	

//10.5
// float Poly_Coefficient[12][4]  = {-2969.23772, 2428.94395, -756.75981, 36.98421, -727.00278, 573.48725, -175.05208, 10.90403, -503.89839, 379.85659, -92.16315, 1.18609, -1434.80669, 1084.34011, -266.04758, 4.97883, -3740.12092, 2834.27802, -706.51514, 64.74360, -75.90890, 58.28145, -15.13242, 2.04268, -671.01497, 504.47928, -117.70333, 11.45603, -103.37117, 78.75421, -18.13971, 1.73936, -226.22809, 172.36905, -43.22514, 3.98215, -579.93848, 441.99249, -110.76262, 10.23112, 1002.44738, -751.18409, 184.48720, 31.33265, 39.70670, -29.98927, 7.45928, 0.77673};
    // Q=diag([2 1 50 250 3000 1]);
    // R=diag([15 4]);
	//好
 float Poly_Coefficient[12][4] = {-1054.29009, 897.17949, -300.39033, 10.24151, -358.65714, 284.42736, -83.87479, 5.24555, -80.54457, 60.46272, -14.73060, -0.72226, -277.78399, 209.48646, -51.74162, -1.19321, -1320.00788, 1000.29448, -250.90686, 23.50797, -52.73585, 40.39285, -10.43662, 1.23315, -1133.61702, 851.39937, -203.55656, 18.12949, -134.76995, 102.29154, -24.30619, 2.16750, -260.28510, 196.41486, -48.53919, 4.19676, -704.09611, 531.45447, -131.32041, 11.36997, 691.74977, -516.72698, 126.43761, 16.84215, 46.48765, -34.92827, 8.61994, 0.27783};
// float Poly_Coefficient[12][4] = {-1037.91998, 889.29178, -302.42403, 10.02287, -362.08098, 287.51257, -85.65020, 5.33908, -79.03033, 59.62016, -14.57048, -1.32358, -257.45204, 195.18511, -48.60409, -2.01985, -432.57982, 332.94806, -87.80824, 10.23496, -22.17278, 17.32302, -4.78014, 0.78016, -141.97713, 100.69460, -21.15787, 2.66242, -5.60592, 3.36457, -0.39651, 0.11380, -53.45829, 40.50057, -10.18684, 0.96428, -116.96419, 88.59696, -22.28862, 2.12966, 65.26644, -49.50507, 12.55819, 7.86946, 7.30610, -5.55051, 1.41322, 0.39301};
//float Poly_Coefficient[12][4] = {-1042.25322, 892.54244, -303.21607, 10.08893, -362.61389, 287.91440, -85.74801, 5.34730, -80.15880, 60.46513, -14.77759, -1.30653, -260.00878, 197.09982, -49.07340, -1.98121, -419.70272, 323.32875, -85.40674, 10.00407, -21.44193, 16.77460, -4.64425, 0.76790, -153.02099, 109.12903, -23.19944, 2.84666, -6.86613, 4.34832, -0.63240, 0.13566, -56.80579, 43.04611, -10.82018, 1.01968, -124.55766, 94.37474, -23.72603, 2.25554, 66.27792, -50.28070, 12.75386, 7.37455, 7.51287, -5.70797, 1.45249, 0.37623};

//  Q=diag([200 10 400 850 4000 1]); %theta dot_theta x dot_x  phi dot_phi 
//  R=diag([45 23]);
// float Poly_Coefficient[12][4] = {-1175.50066, 987.95903, -322.95790, 11.69234, -374.06103, 295.92161, -86.83103, 5.41615, -148.16884, 111.24088, -27.18480, -0.58396, -369.07725, 278.11625, -68.75275, -0.13930, -853.71619, 652.50117, -166.77233, 16.70010, -33.61874, 26.11343, -6.97592, 0.95696, -997.38917, 751.87834, -180.79135, 16.68895, -117.60152, 89.67685, -21.45677, 2.00289, -325.73017, 247.00401, -61.52954, 5.43668, -633.38257, 480.54552, -119.72992, 10.60582, 435.68643, -327.41762, 80.94855, 6.15367, 35.37824, -26.72800, 6.65583, 0.06295};

//对Q矩阵中的x，xb奖励增大尝试   有一定的抑制效果但是解决不了，而且响应变慢了
// float Poly_Coefficient[12][4] = {-1707.65338, 1412.39152, -450.77466, 19.57350, -474.29766, 374.16020, -112.10128, 7.06856, -211.70720, 158.85323, -38.55688, -0.15809, -643.57320, 484.63599, -119.11801, 0.79656, -1695.88258, 1286.62893, -321.74574, 30.09703, -52.28436, 40.22139, -10.45367, 1.35318, -1981.33350, 1502.02476, -359.14216, 31.67022, -281.95294, 216.94520, -51.42168, 4.52567, -570.11206, 431.39096, -106.59659, 9.25390, -1490.70509, 1128.40653, -278.71085, 24.22040, 1167.17077, -872.62077, 213.22094, 9.72247, 78.28733, -58.94782, 14.53487, -0.22800};
// float Poly_Coefficient[12][4] = {-1525.02116, 1268.93370, -409.59547, 16.99253, -441.32322, 348.59897, -104.29445, 6.56099, -170.50298, 127.99377, -31.09926, -0.42181, -531.28447, 400.34638, -98.56365, -0.00051, -1381.71507, 1049.51977, -263.55379, 25.10187, -46.84563, 36.07984, -9.42100, 1.23705, -1514.57243, 1145.14134, -273.19444, 24.32078, -205.96103, 157.98017, -37.33071, 3.31544, -426.08356, 322.34687, -79.71944, 6.94491, -1117.84623, 845.97501, -209.13499, 18.24150, 792.74876, -593.21630, 145.26058, 9.93401, 59.12350, -44.53790, 11.00201, -0.05745};
// float Poly_Coefficient[12][4] = {-1986.11692, 1631.65772, -513.46664, 23.56697, -525.62246, 414.11869, -124.17764, 7.87226, -277.56424, 208.27224, -50.52153, 0.32212, -821.52722, 618.47855, -151.79868, 2.19661, -1801.16177, 1367.40188, -341.95337, 32.07822, -49.84719, 38.45429, -10.05103, 1.36515, -2311.09154, 1756.19091, -419.75862, 37.04995, -338.50220, 261.30121, -61.85133, 5.45100, -692.12657, 524.29517, -129.64687, 11.29600, -1799.95926, 1364.04391, -337.12787, 29.40251, 1353.14834, -1012.25414, 247.39480, 6.89148, 90.75089, -68.39807, 16.87234, -0.43101};
// float Poly_Coefficient[12][4] = {-1467.97002, 1221.51685, -393.42386, 16.23661, -429.19502, 339.01002, -100.79773, 6.37285, -160.95905, 120.76962, -29.36409, -0.31188, -503.37293, 379.12231, -93.33260, 0.21442, -1140.25244, 867.75152, -218.92336, 21.25551, -40.44775, 31.25469, -8.22833, 1.11594, -1578.86913, 1193.88157, -285.11128, 25.43176, -206.55236, 158.49221, -37.43839, 3.34198, -433.26222, 327.90140, -81.15194, 7.08182, -1140.97870, 863.80845, -213.70506, 18.67335, 727.26349, -544.72478, 133.60204, 6.86210, 59.92364, -45.16759, 11.16912, -0.17350};
// float Poly_Coefficient[12][4] = {-1357.17924, 1134.94064, -368.81236, 14.66515, -410.16461, 324.32124, -96.31682, 6.07982, -137.02947, 102.86674, -25.03449, -0.46472, -437.59712, 329.80140, -81.29915, -0.25733, -970.73089, 739.75208, -187.47988, 18.54772, -37.02647, 28.64689, -7.57854, 1.04441, -1250.16183, 943.16232, -224.67948, 20.25527, -155.97717, 119.35040, -28.08937, 2.53614, -336.70581, 254.83504, -63.13181, 5.53147, -889.33694, 673.30266, -166.74551, 14.63199, 533.61115, -400.08234, 98.35824, 6.88266, 47.26315, -35.64264, 8.82971, -0.05197};
// float Poly_Coefficient[12][4] = {-1521.47245, 1257.52156, -397.87290, 17.14050, -432.71672, 341.31571, -100.26839, 6.42512, -195.58801, 146.62159, -35.69650, 0.19835, -554.61615, 417.29533, -102.70808, 1.54922, -812.91708, 621.72267, -158.60014, 16.10420, -28.19515, 22.04459, -5.96394, 0.90719, -1953.58718, 1479.90512, -354.42411, 31.74561, -242.52503, 186.62771, -44.07899, 3.98050, -576.49942, 436.96444, -108.37516, 9.50989, -1389.47486, 1053.58920, -261.21676, 22.95700, 748.03639, -561.52378, 138.19695, 1.14416, 71.00871, -53.61120, 13.28699, -0.50273};
// float Poly_Coefficient[12][4] = {-1347.83403, 1114.76746, -350.60515, 14.71934, -395.19575, 311.93664, -89.90887, 5.82258, -156.52886, 117.16815, -28.56015, 0.39757, -453.64105, 340.76124, -83.76721, 1.80155, -644.44455, 494.14558, -126.95100, 13.12696, -24.62146, 19.32508, -5.27425, 0.79225, -2365.14466, 1789.47840, -430.80101, 38.46330, -254.20854, 195.23509, -46.31697, 4.22046, -582.51262, 441.43562, -109.55064, 9.59313, -1447.93870, 1097.68470, -272.37323, 23.89050, 845.08119, -634.47884, 156.31705, -2.01479, 83.67807, -63.14290, 15.65360, -0.74951};
// float Poly_Coefficient[12][4]= {-1321.91208, 1108.40403, -362.33282, 14.12707, -405.02311, 320.41252, -95.35425, 5.99714, -128.45912, 96.46226, -23.47297, -0.59180, -414.67405, 312.65755, -77.11464, -0.59877, -1067.11501, 812.01590, -205.16849, 20.02772, -40.86164, 31.52382, -8.28320, 1.10321, -1152.38222, 868.44086, -206.68896, 18.63190, -144.64610, 110.49676, -26.01031, 2.34326, -309.04434, 233.77116, -57.88205, 5.06333, -816.63416, 617.91254, -152.94617, 13.39943, 529.86517, -396.97750, 97.49652, 8.94914, 44.48012, -33.52371, 8.29999, 0.04732};
// float Poly_Coefficient[12][4] = {-1426.28647, 1190.14631, -385.76953, 15.60285, -423.01408, 334.31016, -99.64174, 6.27409, -150.45585, 112.92369, -27.45299, -0.46706, -475.42047, 358.22494, -88.23712, -0.20040, -1253.94825, 952.96672, -239.75675, 22.98789, -44.77559, 34.49944, -9.02208, 1.18180, -1455.43617, 1099.44585, -262.39201, 23.38826, -191.64456, 146.82868, -34.69883, 3.08968, -397.47034, 300.64221, -74.36192, 6.47684, -1047.27721, 792.40827, -195.92817, 17.08703, 724.78567, -542.46689, 132.91828, 9.29678, 56.34080, -42.43948, 10.48757, -0.05394};
// float Poly_Coefficient[12][4] = {-1386.84296, 1159.31540, -376.99220, 15.04495, -416.21154, 329.05950, -98.03842, 6.16965, -141.97151, 106.57724, -25.91893, -0.52019, -452.08938, 340.73354, -83.97113, -0.36500, -1181.27375, 898.14610, -226.31037, 21.84075, -43.28255, 33.36438, -8.74039, 1.15220, -1329.66118, 1003.55644, -239.26278, 21.41411, -172.23198, 131.81669, -31.10699, 2.78124, -361.01617, 273.07369, -67.56884, 5.89471, -952.10655, 720.40384, -178.19407, 15.56689, 642.13752, -480.78590, 117.90460, 9.22359, 51.40983, -38.73318, 9.57838, -0.01021};
// float Poly_Coefficient[12][4] = {-1129.35012, 956.15828, -317.34351, 11.36576, -371.92685, 294.72251, -87.06434, 5.46962, -96.50996, 72.45864, -17.65135, -0.71936, -315.43542, 237.92206, -58.78103, -0.99219, -1065.61574, 809.81389, -204.36169, 19.73687, -43.97896, 33.82219, -8.82909, 1.11143, -1047.47137, 787.14280, -187.57552, 16.88708, -122.59636, 93.21381, -21.99873, 1.98478, -266.79191, 201.56475, -49.87821, 4.34476, -684.91896, 517.60669, -128.05838, 11.17315, 510.33713, -381.96731, 93.74321, 11.22188, 41.17032, -30.98980, 7.66737, 0.15452};
//对sita和sita_dot奖励增大尝试
// float Poly_Coefficient[12][4] = {-1177.39528, 980.29774, -314.65229, 10.87062, -363.44052, 286.45688, -82.88149, 5.02242, -114.52734, 85.89335, -21.02177, -0.00212, -360.93684, 270.72002, -66.41246, 0.44255, -1606.01260, 1223.18882, -309.47563, 28.80917, -55.09828, 42.52472, -11.11572, 1.29924, -1497.57742, 1135.97536, -279.14047, 25.45658, -202.31332, 154.86604, -38.24160, 3.52383, -277.29019, 210.85361, -52.80645, 4.64496, -782.62673, 595.38756, -149.18923, 13.13871, 1135.18771, -851.05209, 209.64098, 6.80006, 55.17942, -41.59301, 10.32302, 0.05772};
// float Poly_Coefficient[12][4] = {-1376.88737, 1123.89847, -345.29962, 12.51007, -389.82422, 305.05517, -86.24789, 5.06062, -135.70125, 102.09870, -25.16306, 0.51062, -420.60840, 315.30000, -77.27103, 1.66220, -1470.60235, 1131.53018, -291.40378, 28.08015, -45.71168, 35.82734, -9.63030, 1.19813, -1199.74455, 923.42182, -233.09429, 22.87097, -174.06346, 135.53022, -34.68464, 3.47321, -183.51338, 141.49788, -36.23415, 3.34907, -543.37638, 419.32498, -107.51814, 9.94619, 1037.52144, -784.06555, 195.57173, 2.50427, 44.64702, -33.93402, 8.53638, 0.06703};

//9.30
// float Poly_Coefficient[12][4] = {-530.56254, 522.04951, -201.35625, 5.81346,
// 	-113.71648, 106.71366, -41.22171, 1.68544,
// 		-96.25677, 83.18131, -21.24060, -1.83299, 
// 		-197.99263, 171.28794, -44.30590, -3.73680,
// 		-157.96645, 152.23392, -83.02290, 33.04249,
// 		26.54151, -22.98194, 1.25121, 4.06498,
// 		659.16889, -580.30264, 187.96273, -3.74313, 
// 		125.51825, -104.88944, 30.97526, -0.77725, 
// 	127.31478, -96.58071, 20.42665, 1.24595, 
// 	264.35282, -199.69374, 42.30404, 2.41148,
// 	404.16706, -386.42653, 156.14425, 25.47839, 
// 	-33.79435, 20.75503, 2.41196, 1.89930};		

//    Q=diag([20 1 100 400 1800 1]);
//    R=[14 0;0,5];                %T Tp
// float Poly_Coefficient[12][4] = 
// {-245.2415,290.0232,-167.9004,0.0761,
//  0.1713,0.4242,-13.3698,0.0570,
//  -13.9550,11.3405,-2.9107,-2.4327,
//  -30.4284,24.2259,-6.6633,-6.2953,
//  -264.6721,315.2382,-154.8384,39.2094,
//  -30.0836,37.7845,-19.2775,6.3949,
//  453.1107,-418.7764,147.8488,8.3875,
//  27.5455,-24.8131,4.4844,0.5471,
//  -53.0294,55.2429,-21.7816,2.6050,
//  -133.5842,138.7853,-54.7394,6.2105,
//  816.2537,-784.2204,286.6425,19.7176,
//  130.1190,-127.0704,47.4577,1.6458};
vmc_leg_t right;//右腿

extern INS_t INS;

extern vmc_leg_t left;

chassis_t chassis_move_balance;
extern c_fbpara_t  C_data;

float jump_time_r;
extern float jump_time_l;
												
pid_type_def LegR_Pid;//右腿的腿长pd
pid_type_def Tp_Pid;//防劈叉补偿pd
pid_type_def Turn_Pid;//转向pd
pid_type_def Roll_Pid;//横滚角补偿pd
pid_type_def Wheel_Pid; //3508PID
pid_type_def Wz_Pid;
extern pid_type_def buffer_pid;

extern shoot_control_t shoot_control;          
uint8_t filter_flag =0;
/**
* @description: 数据融合
* @param {dt} 时间步长
* @param {acc} 加速度测量值
* @param {speed} 速度测量值
* @param {vel_esti} 指向估计速度的指针
* @return {*}
*/
float Rz = 1; 
float Qz = 0.03;
uint8_t W_cplt_flag=0;
float angle=0;
void speed_est(const float dt, const float acc, const float speed, float *vel_esti)
{
	static float _x = 0;
	static float _Pz = 1;

	float H = 1;
	float A = 1;
	float B = dt;
	float _x_est;
	float _p_est;
	float K;

	// 预测步骤

	_x_est = A * _x + B * acc;
	_p_est = _Pz + Qz; // A * _P * A_T + Q

	// 更新步骤
	K = _p_est / (_p_est + Rz);
	_x = _x_est + K * (speed - _x_est);
	_Pz = (1 - K) * _p_est;   

	*vel_esti = _x;
}
// 初始化静态变量
//static float Rz = 1.0f;       // 初始测量噪声方差
//static float Qz = 0.03f;      // 固定过程噪声方差
//static float _x = 0.0f;       // 状态估计
//static float _Pz = 1.0f;      // 估计协方差

//void speed_est(const float dt, const float acc, const float speed, float *vel_esti) {
//    const float ALPHA = 0.03f; // 平滑系数 (5%)
//    const float MAX_ERR = 3.5f; // 最大允许残差

//    float H = 1.0f, A = 1.0f, B = dt;
//    float _x_est, _p_est, K;

//    // 预测步骤
//    _x_est = A * _x + B * acc;
//    _p_est = _Pz + Qz;

//    // 动态更新Rz
//    float residual = speed - _x_est;
//    residual = fmaxf(fminf(residual, MAX_ERR), -MAX_ERR); // ???
//    Rz = (1 - ALPHA) * Rz + ALPHA * residual * residual;

//    // 更新步骤
//    K = _p_est / (_p_est + Rz);
//    _x = _x_est + K * residual;
//    _Pz = (1 - K) * _p_est;

//    *vel_esti = _x;
//}

extern float LQR_K_L[12];
float kf_fusion;
float forward_acc;

float data_fusion(void)//速度融合调用
	{
	float fusion;
	forward_acc = -INS.MotionAccel_b[0];
	float speed;
	//速度方向 修改
	//	chassis->v = ((chassis_move_balance.wheel_motor[0].speed) - (chassis_move_balance.wheel_motor[1].speed))/2 ;	

	speed = ((chassis_move_balance.wheel_motor[0].speed) - (chassis_move_balance.wheel_motor[1].speed))/2 ;
	
	speed_est(0.001,forward_acc,speed,&fusion);

	return fusion;
}


uint32_t CHASSR_TIME=1;	//1ms
float yaw_sen;
float pitch_sen;
float mode_rc;
float fire_mode;

void ChassisR_task(void)
{
	//保持平衡
	chassis_move_balance.leg_set=0.13; 
	chassis_move_balance.turn_set = 0;
	chassis_move_balance.x_set = 0;
	chassis_move_balance.v_set = 0;
	chassis_move_balance.recover_flag = 0;
	chassis_move_balance.roll_set = 0.0005;
	shoot_control.shoot_send_flag = 0;
	chassis_move_balance.target_x = 0;
	while(INS.ins_flag==0)
	{//等待加速度收敛
	  osDelay(1);	
	}
	//电机初始化
	//包括了右边关节电机的id,mode初始化，以及电机的使能；
	  ChassisR_init(&chassis_move_balance,&right,&LegR_Pid,&Wheel_Pid);
	  Pensation_init(&Roll_Pid,&Tp_Pid,&Turn_Pid,&Wz_Pid);//pid初始化	
	  shoot_init();	
    //chassis_move_balance.leg_set = 0.127f;//原始腿长    腿长限制在0.127------0.32  会比设定高1-2cm
	while(1)
	{	
		 //等待电机使能
		chassis_move_balance.DUBS_ON=toe_is_error(DBUS_TOE);//0:正常 1:异常
//		c_transmit_date(&hcan1,chassis_move_balance.chassis_RC->rc.s[0],
//		chassis_move_balance.chassis_RC->rc.ch[3],
//		chassis_move_balance.chassis_RC->rc.ch[2],
//		chassis_move_balance.chassis_RC->rc.s[1],
//		INS.Yaw,chassis_move_balance.chassis_RC->key.v,
//		chassis_move_balance.DUBS_ON);

		if(chassis_move_balance.chassis_RC->rc.s[1] == 1){
	
		fire_mode = 1;		
		}
		if(chassis_move_balance.chassis_RC->rc.s[1] == 3){
		
		fire_mode = 0;		
		}
				
        //板间通信
	    c_transmit_date(yaw_sen, pitch_sen,mode_rc,shoot_control.shoot_send_flag, 11);
		
		osDelay(1);
				
		float dt = (float)CHASSR_TIME/1000.0f;//1/1000	

		if( chassis_move_balance.chassis_RC->rc.s[0] == 3)//右拨杆中间，使能
		{
		   chassis_move_balance.start_flag=1;
			mode_rc  = 1; 
		}
	
		if( chassis_move_balance.chassis_RC->rc.s[0] == 2)//右拨杆最下，失能
		{
			
		   chassis_move_balance.start_flag=0;
			mode_rc  = 0;   //无力模式
		}

		if( chassis_move_balance.chassis_RC->rc.s[0] == 1)
		{
			chassis_move_balance.w_flag  = 1;//底盘不跟随云台，小陀螺模式

		}else{
		chassis_move_balance.w_flag  = 0;//底盘跟随云台
		}

		//倒地检测
	    chassis_move_balance.recover_flag = recover_detect(&chassis_move_balance);	

		//CHASSR_TIME=1;
		
	if(chassis_move_balance.start_flag==1)
   {	
	   if(chassis_move_balance.chassis_RC->key.v & CHASSIS_FRONT_KEY){
	
	        chassis_move_balance.target_v = 2;	
	    }else if(chassis_move_balance.chassis_RC->key.v & CHASSIS_BACK_KEY){
			
			chassis_move_balance.target_v = -2;
	     }
   
		// if((float)chassis_move_balance.chassis_RC->rc.ch[1]>=440)      (float)chassis_move_balance.chassis_RC->rc.ch[1]=440;
		// else if((float)chassis_move_balance.chassis_RC->rc.ch[1]<=440) (float)chassis_move_balance.chassis_RC->rc.ch[1]=-440;
	
		chassis_move_balance.target_v=
		                            //    0;
 		                            ((float)chassis_move_balance.chassis_RC->rc.ch[1])*(0.0050f);
		slope_following(&chassis_move_balance.target_v,&chassis_move_balance.v_set,0.004f);	//斜坡函数
			
//	    chassis_move_balance.v_set=((float)chassis_move_balance.chassis_RC->rc.ch[1])*(0.00450f);//???????0	
	
//x位控
        // chassis_move_balance.target_x=chassis_move_balance.target_x+((float)chassis_move_balance.chassis_RC->rc.ch[1])*(0.000005f);
        chassis_move_balance.x_set = 
		                            // 0.5f;................................................................................................................................
		                            chassis_move_balance.x_set+chassis_move_balance.v_set*(float)CHASSR_TIME*5.0f/1000.0f+0.000009f;
									// chassis_move_balance.x_set+((float)chassis_move_balance.chassis_RC->rc.ch[1])*(0.0000265f)+chassis_move_balance.v_set*(float)CHASSR_TIME*2.2f/1000.0f;
		
									// slope_following(&chassis_move_balance.target_x,&chassis_move_balance.x_set,0.012f);//斜坡函数，起一个补偿作用
//刹车补偿       
		// if(chassis_move_balance.target_v<0.0125f && chassis_move_balance.target_v>-0.0125f&&right.leg_flag==0&&left.leg_flag==0)
		// {
		// 	chassis_move_balance.x_set=chassis_move_balance.x_set+(left.L0*sinf(left.theta)+right.L0*sinf(right.theta))/2.0f; 
		// 	// chassis_move_balance.x_set=0.0f;
        //     // filter_flag = 1;
		// }	
        
		yaw_sen = chassis_move_balance.chassis_RC->rc.ch[2]*(0.00003f) - chassis_move_balance.chassis_RC->mouse.x * YAW_MOUSE_SEN;//???????0
	
		pitch_sen =((float)chassis_move_balance.chassis_RC->rc.ch[3])*(0.00003f) + chassis_move_balance.chassis_RC->mouse.y * PITCH_MOUSE_SEN;
	
//       chassis_move_balance.turn_set = chassis_move_balance.turn_set+(float)chassis_move_balance.chassis_RC->rc.ch[2]*(-0.00006f);
	
//往右大于0		
	if((shoot_control.shoot_rc->key.v & CHASSIS_LEG_KEY) && 
        !(shoot_control.last_key & CHASSIS_LEG_KEY)){

	   	chassis_move_balance.leg_set = chassis_move_balance.leg_set + 0.1; 
	}
   	chassis_move_balance.leg_set = chassis_move_balance.leg_set+(((float)chassis_move_balance.chassis_RC->rc.ch[0])*(0.0000037f)); 
		//腿长限幅
		mySaturate(&chassis_move_balance.leg_set,0.130f,0.32f);
	
		if(fabsf(chassis_move_balance.last_leg_set-chassis_move_balance.leg_set)>0.0007f)
	{
				//遥控器控制腿长在变化
				right.leg_flag=1;	//为1标志着遥控器在控制腿长伸缩，根据这个标志可以不进行离地检测，因为当腿长在主动伸缩时，离地检测会误判为离地了
				left.leg_flag=1;	 			
}	
	chassis_move_balance.last_leg_set=chassis_move_balance.leg_set;
//小陀螺
	if(chassis_move_balance.w_flag == 1)  
	{
		chassis_move_balance.Wz_target= 1.3;
		chassis_move_balance.v_set = 0;
		chassis_move_balance.x_set = 0;
		chassis_move_balance.target_x=0;
	}else {
	chassis_move_balance.w_flag=0;	
	chassis_move_balance.Wz_set=0;
	}		
}	
//更新数据
	    chassisR_feedback_update(&chassis_move_balance,&right,&INS);

//	 chassis_move_balance.turn_set = 0;
//	 chassis_set_mode(&chassis_move_balance);

//控制计算
	    chassisR_control_loop(&chassis_move_balance,&right,&INS,LQR_K_L,&LegR_Pid);	
//		mit_ctrl(&hcan1,0X06,0,1,0,1,0);

//足电机控制
		if(chassis_move_balance.start_flag==1)	
		{
			mit_ctrl(&hcan1,0x08, 0.0f, 0.0f,0.0f, 0.8f,right.torque_set[1]);//right.torque_set[1]
			osDelay(CHASSR_TIME);
			mit_ctrl(&hcan1,0x06, 0.0f, 0.0f,0.0f, 0.8f,right.torque_set[0]);//right.torque_set[0]
			osDelay(CHASSR_TIME);	
			//顺时针为正
			chassis_move_balance.wheel_motor[0].given_current = (chassis_move_balance.wheel_motor[0].wheel_T/0.000396211f);
    	   
			CAN_cmd_chassis(-chassis_move_balance.wheel_motor[0].given_current);
            osDelay(CHASSR_TIME);
		}

//			mit_ctrl(&hcan1,0x08, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right.torque_set[1]
//			osDelay(CHASSR_TIME);
//			mit_ctrl(&hcan1,0x06, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right.torque_set[0]
//			osDelay(CHASSR_TIME);
//			CAN_cmd_chassis(0);
		 else if(chassis_move_balance.start_flag==0)
		{
			mit_ctrl(&hcan1,0x08, 0.0f, 0.0f,0.0f, 0.8f,0.0f);//right.torque_set[1]
			osDelay(CHASSR_TIME);
			mit_ctrl(&hcan1,0x06, 0.0f, 0.0f,0.0f, 0.8f,0.0f);//right.torque_set[0]
			osDelay(CHASSR_TIME);
			CAN_cmd_chassis(0);
			osDelay(CHASSR_TIME);
		}
		//跳跃PID
		// else if(chassis_move_balance.start_flag==0&&chassis_move_balance.help_jump_flag ==1)
		// //(不在意,不在意, float pos, float vel,float kp, float kd, float torq)
		// {
        //   	mit_ctrl(&hcan1,0x08, 0.0f, 0.0f,0.0f, 0.8f,15.0f);
		// 	osDelay(CHASSR_TIME);
		// 	mit_ctrl(&hcan1,0x06, 0.0f, 0.0f,0.0f, 0.8f,15.0f);
		// 	osDelay(CHASSR_TIME);
		// 	CAN_cmd_chassis(0);
		// 	osDelay(CHASSR_TIME);
		// }
  }
}

//底盘初始化
void ChassisR_init(chassis_t *chassis,vmc_leg_t *vmc,pid_type_def *legr,pid_type_def *wheel)
{
	
	chassis->chassis_RC = get_remote_control_point();//获取遥控指针

   const static float legr_pid[3] = 
                                    //   {0,0,0};
                                   {200.0f,0.0f,500.0f};//{LEG_PID_KP, LEG_PID_KI,LEG_PID_KD};

   const static fp32 power_pid[3] = {POWER_PID_KP, POWER_PID_KI, POWER_PID_KD};
  
   const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
	
    //右边关节电机的初始化
	joint_motor_init(&chassis->joint_motor[0],6,MIT_MODE);//发送id为6
	joint_motor_init(&chassis->joint_motor[1],8,MIT_MODE);//发送id为8
	
	VMC_init(vmc);//给杆长赋值
	
	//腿长pid初始化
	PID_init(legr, PID_POSITION,legr_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);

	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hcan1,chassis->joint_motor[1].para.id,chassis->joint_motor[1].mode);
	  osDelay(1);
	}
	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hcan1,chassis->joint_motor[0].para.id,chassis->joint_motor[0].mode);
	  osDelay(1);
	}	
	for(int m=0;m<2;m++)
    {
       
       PID_init(&chassis->motor_speed_pid[m], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
	
	   PID_init(&chassis->buffer_pid, PID_POSITION,power_pid,POWER_PID_MAX_OUT, POWER_PID_MAX_IOUT);
}

float roll_pid[3] = 
                    {40,0,0.1};   //测试使用
                    // {ROLL_PID_KP, ROLL_PID_KI,ROLL_PID_KD};
float tp_pid[3]   = 
//                  {0,0,0};
                    {TP_PID_KP, TP_PID_KI, TP_PID_KD};//防劈叉PID
float turn_pid[3] = 
//                  {0,0,0};
                    {TURN_PID_KP, TURN_PID_KI, TURN_PID_KD};
float wz_pid[3]   = 
//                  {0,0,0};
                    {WZ_PID_KP, WZ_PID_KI, WZ_PID_KD};
//PID初始化
void Pensation_init(pid_type_def *roll,pid_type_def *Tp,pid_type_def *turn,pid_type_def *wz)
{//补偿pid初始化：横滚角补偿、防劈叉补偿、偏航角补偿
    
//	const static float roll_pid[3] = {ROLL_PID_KP, ROLL_PID_KI,ROLL_PID_KD};
//	const static float tp_pid[3] = {TP_PID_KP, TP_PID_KI, TP_PID_KD};
//	const static float turn_pid[3] = {TURN_PID_KP, TURN_PID_KI, TURN_PID_KD};	
//	const static fp32 power_pid[3] = {POWER_PID_KP, POWER_PID_KI, POWER_PID_KD};

//	横滚角补偿
	PID_init(roll, PID_POSITION, roll_pid, ROLL_PID_MAX_OUT, ROLL_PID_MAX_IOUT);
//  防劈叉补偿
	PID_init(Tp, PID_POSITION, tp_pid, TP_PID_MAX_OUT,TP_PID_MAX_IOUT);
//  偏航角补偿
	PID_init(turn, PID_POSITION, turn_pid, TURN_PID_MAX_OUT, TURN_PID_MAX_IOUT);
//  Wz侧补偿	
	PID_init(wz, PID_POSITION, wz_pid, WZ_PID_MAX_OUT, WZ_PID_MAX_IOUT);
//	PID_init(&buffer_pid, PID_POSITION,power_pid,POWER_PID_MAX_OUT, POWER_PID_MAX_IOUT);

}

//底盘数据反馈更新
void chassisR_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins)
{
	
	//get remote control point
    //调用获取遥控器指针，得到当前各个通道的值 
    chassis->chassis_RC = get_remote_control_point();
	
    vmc->phi1=pi/2.0f+chassis->joint_motor[0].para.pos;
	vmc->phi4=pi/2.0f+chassis->joint_motor[1].para.pos;
		
	chassis->myPithR=ins->Pitch;
	chassis->myPithGyroR=ins->Gyro[1];
	
	chassis->relative_angle = chassis->yaw_motor_angle;
	
	chassis->total_yaw=ins->YawTotalAngle;
	chassis->roll=ins->Roll;
	chassis->theta_err=0.0f-(vmc->theta+left.theta);	//两腿theta之差
	
	chassis->wheel_motor[0].vel = chassis->wheel_motor[0].speed_rpm * CHASSIS_MOTOR_RPM_TO_OMG_SEN;  
	chassis->wheel_motor[0].speed = 0.0004998609952f * chassis->wheel_motor[0].speed_rpm;  
//	chassis->wheel_motor[0].wheel_T = CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN * chassis->wheel_motor[0].given_current;

//	chassis->Wz = (chassis->wheel_motor[0].vel)+(chassis->wheel_motor[1].vel )/2;
//  chassis->Wz = chassis->total_yaw;

//	chassis->v = ((chassis_move_balance.wheel_motor[0].speed) - (chassis_move_balance.wheel_motor[1].speed))/2 ;		
//	chassis->x = chassis->x + chassis->v*((float)1/1000.0f);	
	
	chassis->v = data_fusion();
	chassis->x = chassis->x + chassis->v*((float)CHASSR_TIME/1000.0f);
	
//	
	//倒地检测	
	//根据pitch角度判断倒地自起是否完成
	if(ins->Pitch<(3.1415926f/15.5f)&&ins->Pitch>(-3.1415926f/15.5f))
	{
		chassis->recover_flag=0;
		// chassis_move_balance.target_x=0;
	}	
}

void dm4310_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len)
{ 
	if(data_len==8)
	{//返回的数据有8个字节
	  motor->para.id = (rx_data[0])&0x0F;
	  motor->para.state = (rx_data[0])>>4;
	  motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	  motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	  motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	  motor->para.pos = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
	  motor->para.vel = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12); // (-30.0,30.0)
	  motor->para.tor = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12);  // (-10.0,10.0)
	  motor->para.Tmos = (float)(rx_data[6]);
	  motor->para.Tcoil = (float)(rx_data[7]);
	}
}
	
uint8_t right_flag=0;
extern uint8_t left_flag;
int jump_right_flag = 0;

void chassisR_control_loop(chassis_t *chassis,vmc_leg_t *vmcr,INS_t *ins,float *LQR_K,pid_type_def *leg)
{
	
	VMC_calc_1_right(vmcr,ins,((float)CHASSR_TIME)*3.0f/1000.0f);
     
    for(int i=0;i<12;i++)
   {
	LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0],vmcr->L0);	
   }
//	chassis->turn_T=PID_Calc(&Turn_Pid, chassis->total_yaw, chassis->turn_set);
//  chassis->turn_T=Turn_Pid.Kp*(chassis->turn_set-chassis->total_yaw)-Turn_Pid.Kd*ins->Gyro[2];
//	chassis->roll_f0=PID_Calc(&Roll_Pid, chassis->roll,chassis->roll_set);
//	chassis->roll_f0=Roll_Pid.Kp*(chassis->roll_set-chassis->roll)-Roll_Pid.Kd*ins->Gyro[1];
//	chassis->leg_tp=PID_Calc(&Tp_Pid, chassis->theta_err,0.0f);

//    if(chassis->w_flag==1){	
// 	   chassis->turn_T=Turn_Pid.Kp*(chassis->Wz_set-0)-Turn_Pid.Kd*ins->Gyro[2];
// 	}
// 	else{
// 		chassis->turn_T=Turn_Pid.Kp*(chassis->relative_angle-0)-Turn_Pid.Kd*ins->Gyro[2];
//       //chassis->turn_T = - PID_calc(&Turn_Pid,chassis->relative_angle,0);
// 	}

   if(chassis->w_flag==1){	
		slope_following(&chassis_move_balance.Wz_target,&chassis_move_balance.Wz_set,0.008f);	//斜坡函数
	   chassis->turn_T=Turn_Pid.Kp*(chassis->Wz_set-0)-Turn_Pid.Kd*ins->Gyro[2];	//进入小陀螺模式
	   W_cplt_flag=1;
	   angle=0;
	}
	else{
		if(W_cplt_flag==1){
			slope_following(&chassis->relative_angle,&angle,0.007f);	//斜坡函数
			chassis->turn_T=Turn_Pid.Kp*(angle-0)-Turn_Pid.Kd*ins->Gyro[2];
			if(chassis->relative_angle<0.175f&&chassis->relative_angle>-0.175f){				
				W_cplt_flag=0;
			}
		}
		else{
			chassis->turn_T=Turn_Pid.Kp*(chassis->relative_angle-0)-Turn_Pid.Kd*ins->Gyro[2];
		}
		
	}
	//Roll轴补偿
	chassis->roll_f0=Roll_Pid.Kp*(chassis->roll_set-chassis->roll)-Roll_Pid.Kd*ins->Gyro[0];
	//Roll轴补偿的pid限幅
	mySaturate(&chassis->roll_f0,-Roll_Pid.max_out,Roll_Pid.max_out);
	//防劈叉pid计算
	chassis->leg_tp=PID_calc(&Tp_Pid, chassis->theta_err,0.00f);
	
//轮毂电机
//chassis->wheel_motor[0].wheel_T = (LQR_K[0]*(vmcr->theta-0.0f)
//+LQR_K[1]*(vmcr->d_theta-0.0f)
//+LQR_K[2]*(chassis->x-chassis->x_set)
//+LQR_K[3]*(chassis->v-chassis->v_set)
//+LQR_K[2]*(chassis->x_filter-chassis->x_set)
//+LQR_K[3]*(chassis->v_filter2-0.4f*chassis->v_set)
//+LQR_K[4]*(chassis->myPithR-0.01025f) 
//+LQR_K[5]*(chassis->myPithGyroR-0.01f));	

//chassis->wheel_motor[0].wheel_T= ( LQR_K[3]*(chassis->v-0.4f*chassis->v_set) - LQR_K[4]*(chassis->myPithR-0.04f-chassis->phi_set) - LQR_K[5]*(chassis->myPithGyroR-0.0f));		
////右边髋关节输出力矩				
//vmcr->Tp= (LQR_K[6]*0.8*(vmcr->theta-0.0f)	
//+LQR_K[7]*0.7*(vmcr->d_theta-0.0f)
//+LQR_K[8]*(chassis->x-chassis->x_set)
//+LQR_K[9]*(chassis->v-chassis->v_set)
////+LQR_K[8]*(chassis->x_filter-chassis->x_set)
////+LQR_K[9]*(chassis->v_filter2-0.4f*chassis->v_set)
//+LQR_K[10]*(chassis->myPithR-0.01025f)
//+LQR_K[11]*(chassis->myPithGyroR-0.01f));

	//轮毂电机
    // chassis->wheel_motor[0].wheel_T =   0;
 	chassis->wheel_motor[0].wheel_T =   (LQR_K[0]*(vmcr->theta-0.0f)
									    +LQR_K[1]*(vmcr->d_theta-0.0f)
//										+LQR_K[2]*(chassis->x-chassis->x_set)
//										+LQR_K[3]*(chassis->v-chassis->v_set)
                                        // +LQR_K[2]*0.0f
									    +LQR_K[2]*(chassis->x_filter-(chassis->x_set))
									    +LQR_K[3]*(chassis->v_filter2-(chassis->v_set))
//									    +LQR_K[4]*(chassis->myPithR-0.01025f-chassis->phi_set) 
										+LQR_K[4]*(chassis->myPithR-0.0f) 
									    +LQR_K[5]*(chassis->myPithGyroR-0.0f));	
	
	//右边髋关节输出力矩				
	  vmcr->Tp = (LQR_K[6]*(vmcr->theta-0.0f)	
				 +LQR_K[7]*(vmcr->d_theta-0.0f)
//	             +LQR_K[8]*(chassis->x-chassis->x_set)
//	             +LQR_K[9]*(chassis->v-chassis->v_set)
				//  +LQR_K[8]*0.0f
				 +LQR_K[8]*(chassis->x_filter-(chassis->x_set))
				 +LQR_K[9]*(chassis->v_filter2-chassis->v_set)
//			     +LQR_K[10]*(chassis->myPithR-0.01025f-chassis->phi_set)
				 +LQR_K[10]*(chassis->myPithR-0.0f)
				 +LQR_K[11]*(chassis->myPithGyroR-0.0f));
//			 
	      //  chassis->wheel_motor[0].wheel_T = 0;	//测试用
		chassis->wheel_motor[0].wheel_T=chassis->wheel_motor[0].wheel_T - chassis->turn_T;	//T=LQR值+偏航补偿
	
//对轮毂电机输出限幅
	mySaturate(&chassis->wheel_motor[0].wheel_T,-4.2f,4.2f);	
	vmcr->Tp=vmcr->Tp+chassis->leg_tp;//髋关节输出力矩=LQR得到的+防劈叉补偿
	//vmcr->Tp=0.0f;//测试使用
	vmcr->F0=17.2f/arm_cos_f32(vmcr->theta)+PID_calc(leg,vmcr->L0,chassis->leg_set)-chassis->roll_f0;	//f0=前馈(抵消重力)+腿长pd+roll轴补偿
	//vmcr->F0=0.0f;		//测试使用
	if(chassis_move_balance.chassis_RC->rc.s[0]==2)//右拨杆拨至最下边
	{
		chassis_move_balance.target_x=0.0f;
		chassis_move_balance.x_set=0.0f;
	}


//跳跃逻辑
 	if(chassis->chassis_RC->rc.s[1] ==1)//左上拨杆拨至最上边
 	{
 		if(chassis->chassis_RC->rc.ch[4] >500){  //左拨轮往下拨
		
 		    chassis->help_jump_flag =1;		
 		}
	//压缩阶段	
 		  if(chassis->jump_flag_r==0 && chassis->help_jump_flag ==1){
			
 		      chassis->leg_set = 0.130;

 		       if(vmcr->L0<0.17f)
 		     {
 		        jump_time_r++;  
 		      }
 		     if(jump_time_r>=10&&jump_time_l>=10)
 		     {  
 			   jump_time_r=0;
 			   jump_time_l=0;
 			   chassis->jump_flag_r=1;//压缩完毕进入上升加速阶段
			   chassis->jump_flag_l=1;//压缩完毕进入上升加速阶段
		     }			 		 
 		   }

 //上升加速阶段			
 		else if(chassis->jump_flag_r==1&& chassis->help_jump_flag ==1)
 		{		
 			chassis->leg_set = 0.30;

 			 if(vmcr->L0>0.22f)
 			 {
 				jump_time_r++;
 			 }
 			 if(jump_time_r>=10&&jump_time_l>=10)
 			 {  
 				 jump_time_r=0;
 				 jump_time_l=0;
 				 chassis->jump_flag_l=2;//上升完毕进入缩腿阶段
 				 chassis->jump_flag_r=2;
 			 }	 

 			}
//缩腿阶段		
 	 else if(chassis->jump_flag_r==2&& chassis->help_jump_flag ==1)
 		{
 			chassis->leg_set = 0.13;
 			chassis->theta_set=0.0f;
			
 			chassis->x_filter=0.0f;
 			chassis->x_set=chassis->x_filter;
			
 		  if(vmcr->L0<0.17f)
 		  {
 			 jump_time_r++;
 		  }
 		  if(jump_time_r>=5&&jump_time_l>=5)
 		  { 
 			 jump_time_r=0;
 			 jump_time_l=0;
 			 chassis->leg_set=0.130f;
 			 chassis->last_leg_set=0.130f;
 			 chassis->jump_flag_r=0;//缩腿完毕
 		     chassis->jump_flag_l=0;
         chassis->help_jump_flag = 0;	
 		  }
 		}
		
 	else
 	{
 		vmcr->F0=11.2f/arm_cos_f32(vmcr->theta)+PID_calc(leg,vmcr->L0,chassis->leg_set);
 	}
 }	

   right_flag = ground_detectionR(vmcr,ins);//右腿离地检测
	 if(chassis->recover_flag==0)	//倒地自起不需要检测是否离地		
	 { 
		// if((right_flag==1&&left_flag==1&&vmcr->leg_flag==0&&chassis->jump_flag!=1&&chassis->jump_flag2!=1&&chassis->jump_flag!=2&&chassis->jump_flag2!=2)
     
		//||chassis->jump_flag==3)
		if(right_flag==1&&left_flag==1&&vmcr->leg_flag==0)
		{ 
			//当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
			//排除跳跃的压缩阶段、上升阶段、跳跃的缩腿阶段
				chassis->wheel_motor[0].wheel_T=0.0f;
				vmcr->Tp=LQR_K[6]*(vmcr->theta-0.0f)+ LQR_K[7]*(vmcr->d_theta-0.0f);

				chassis->x_filter=0.0f;
				chassis->x_set = chassis->x_filter;
				vmcr->Tp=vmcr->Tp+chassis->leg_tp;			 
		}
		else
		{//没有离地
			vmcr->leg_flag=0;//置为0
							
			// if(chassis->jump_flag_r==0)
			// {//不跳跃的时候需要roll轴补偿					
			//  vmcr->F0=vmcr->F0+chassis->roll_f0;//roll轴补偿取反然后加上去   			
			// }
		}
	 }
	 else if(chassis->recover_flag==1)

	 {
		 vmcr->Tp=0.0f;
		 vmcr->F0=0.0f;
	 }


  //功率控制
    chassis_power_control(chassis);

	mySaturate(&vmcr->F0,-100.0f,100.0f);//限幅

	VMC_calc_2(vmcr);//计算期望的关节输出力矩

//限幅函数，后期要改参数
	mySaturate(&vmcr->torque_set[1],-18.0f,18.0f);	
	mySaturate(&vmcr->torque_set[0],-18.0f,18.0f);	
		 
}
	

void mySaturate(float *in,float min,float max)
{
  if(*in < min)
  {
    *in = min;
  }
  else if(*in > max)
  {
    *in = max;
  }
}



uint8_t recover_detect(chassis_t *chassis)
{	
	if(((chassis->myPithR<((-3.1415926f)/10.5f)&&chassis->myPithR>((-3.1415926f)/2.0f))
					  ||(chassis->myPithR>(3.1415926f/12.8f)&&chassis->myPithR<(3.1415926f/2.0f))))	
	{
				
			chassis->leg_set = 0.130;
	
				return 1;//需要自起		
		}

	else
	  {
		  
	  return 0;	
		  
	   }
}

