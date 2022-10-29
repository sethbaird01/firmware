/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Electronics_data.c
 *
 * Code generated for Simulink model 'Electronics'.
 *
 * Model version                  : 1.131
 * Simulink Coder version         : 9.7 (R2022a) 13-Nov-2021
 * C/C++ source code generated on : Fri Oct 28 19:01:23 2022
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "Electronics.h"

/* Invariant block signals (default storage) */
const ConstB rtConstB = {
  /* Start of '<Root>/Electronics' */
  {
    { 29.07421875, 29.07421875, 29.072265625, 29.072265625 }
                                      /* '<S2>/Fixed Point Sub_boundary_DTC6' */
  }
  /* End of '<Root>/Electronics' */
};

/* Constant parameters (default storage) */
const ConstP rtConstP = {
  /* Computed Parameter: Gain1_Gain_n
   * Referenced by: '<S8>/Gain1'
   */
  { 27158, 27158, 27156, 27156 },

  /* Pooled Parameter (Expression: YData)
   * Referenced by:
   *   '<S19>/Look-Up Table'
   *   '<S21>/Look-Up Table'
   */
  { 0, 201, 402, 603, 804, 1005, 1205, 1406, 1606, 1806, 2006, 2205, 2404, 2603,
    2801, 2999, 3196, 3393, 3590, 3786, 3981, 4176, 4370, 4563, 4756, 4948, 5139,
    5330, 5520, 5708, 5897, 6084, 6270, 6455, 6639, 6823, 7005, 7186, 7366, 7545,
    7723, 7900, 8076, 8250, 8423, 8595, 8765, 8935, 9102, 9269, 9434, 9598, 9760,
    9921, 10080, 10238, 10394, 10549, 10702, 10853, 11003, 11151, 11297, 11442,
    11585, 11727, 11866, 12004, 12140, 12274, 12406, 12537, 12665, 12792, 12916,
    13039, 13160, 13279, 13395, 13510, 13623, 13733, 13842, 13949, 14053, 14155,
    14256, 14354, 14449, 14543, 14635, 14724, 14811, 14896, 14978, 15059, 15137,
    15213, 15286, 15357, 15426, 15493, 15557, 15619, 15679, 15736, 15791, 15843,
    15893, 15941, 15986, 16029, 16069, 16107, 16143, 16176, 16207, 16235, 16261,
    16284, 16305, 16324, 16340, 16353, 16364, 16373, 16379, 16383, 16384 },

  /* Pooled Parameter (Expression: power_loss_grid)
   * Referenced by:
   *   '<S18>/Constant1'
   *   '<S18>/Constant4'
   */
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    38, 67, 92, 115, 136, 156, 177, 199, 222, 247, 274, 303, 335, 368, 404, 440,
    476, 510, 542, 572, 601, 630, 658, 686, 714, 741, 768, 794, 820, 845, 871,
    895, 918, 941, 964, 987, 1010, 1034, 1058, 1083, 1109, 1136, 1163, 1192,
    1222, 1254, 1287, 1323, 1360, 1399, 0, 68, 122, 166, 205, 242, 277, 313, 351,
    390, 432, 477, 527, 580, 637, 697, 758, 817, 875, 929, 980, 1031, 1080, 1129,
    1178, 1226, 1274, 1321, 1367, 1414, 1459, 1505, 1549, 1592, 1636, 1679, 1722,
    1766, 1812, 1859, 1908, 1957, 2009, 2062, 2118, 2176, 2237, 2300, 2368, 2438,
    2513, 0, 94, 166, 226, 278, 326, 372, 418, 466, 516, 570, 628, 691, 759, 832,
    907, 985, 1062, 1135, 1203, 1270, 1335, 1400, 1464, 1528, 1592, 1655, 1718,
    1780, 1842, 1904, 1966, 2027, 2087, 2147, 2208, 2270, 2333, 2399, 2466, 2535,
    2607, 2681, 2758, 2838, 2922, 3009, 3101, 3196, 3298, 3404, 0, 115, 203, 275,
    336, 392, 445, 498, 553, 610, 671, 738, 809, 887, 970, 1057, 1146, 1233,
    1316, 1394, 1471, 1548, 1624, 1699, 1775, 1850, 1925, 1999, 2074, 2148, 2223,
    2297, 2372, 2446, 2521, 2598, 2676, 2756, 2839, 2924, 3012, 3104, 3198, 3297,
    3399, 3506, 3617, 3733, 3855, 3983, 4117, 0, 133, 234, 315, 383, 444, 502,
    559, 617, 679, 744, 815, 893, 976, 1065, 1159, 1255, 1348, 1438, 1523, 1608,
    1692, 1775, 1859, 1943, 2027, 2110, 2194, 2278, 2362, 2447, 2531, 2617, 2703,
    2791, 2880, 2972, 3067, 3165, 3267, 3372, 3481, 3594, 3712, 3834, 3961, 4094,
    4232, 4378, 4529, 4689, 0, 149, 260, 347, 420, 485, 545, 604, 664, 727, 795,
    868, 948, 1034, 1127, 1225, 1324, 1422, 1515, 1605, 1694, 1783, 1872, 1962,
    2051, 2141, 2232, 2322, 2414, 2505, 2597, 2690, 2784, 2881, 2978, 3079, 3183,
    3291, 3402, 3518, 3639, 3763, 3893, 4028, 4168, 4313, 4466, 4624, 4790, 4964,
    5145, 0, 162, 282, 374, 451, 516, 577, 637, 697, 760, 827, 901, 981, 1069,
    1163, 1263, 1363, 1463, 1557, 1649, 1741, 1834, 1927, 2020, 2114, 2209, 2304,
    2399, 2496, 2593, 2692, 2791, 2892, 2995, 3103, 3212, 3327, 3445, 3569, 3697,
    3831, 3970, 4114, 4265, 4420, 4583, 4753, 4929, 5114, 5306, 5508, 0, 174,
    301, 397, 475, 541, 601, 660, 717, 779, 845, 917, 997, 1084, 1178, 1278,
    1379, 1477, 1573, 1667, 1760, 1855, 1950, 2046, 2143, 2241, 2339, 2439, 2539,
    2641, 2744, 2849, 2955, 3064, 3176, 3294, 3417, 3546, 3679, 3820, 3964, 4116,
    4272, 4438, 4607, 4787, 4970, 5163, 5364, 5573, 5794, 0, 185, 317, 416, 494,
    559, 618, 674, 730, 788, 852, 921, 998, 1084, 1177, 1275, 1375, 1472, 1567,
    1660, 1755, 1850, 1946, 2044, 2142, 2241, 2342, 2444, 2548, 2653, 2759, 2868,
    2978, 3091, 3213, 3336, 3466, 3604, 3745, 3894, 4052, 4213, 4385, 4559, 4743,
    4932, 5132, 5340, 5554, 5780, 6016, 0, 195, 331, 432, 510, 573, 630, 681,
    734, 789, 849, 915, 989, 1072, 1162, 1259, 1357, 1452, 1545, 1638, 1732,
    1828, 1924, 2022, 2121, 2222, 2324, 2428, 2533, 2641, 2750, 2862, 2976, 3092,
    3217, 3346, 3483, 3626, 3778, 3934, 4100, 4272, 4452, 4638, 4835, 5036, 5250,
    5467, 5697, 5937, 6184, 0, 204, 345, 446, 522, 583, 636, 685, 732, 783, 838,
    901, 971, 1050, 1138, 1233, 1328, 1421, 1512, 1604, 1697, 1792, 1888, 1986,
    2085, 2187, 2289, 2394, 2501, 2610, 2722, 2835, 2952, 3072, 3197, 3332, 3473,
    3623, 3781, 3948, 4118, 4300, 4488, 4687, 4890, 5104, 5325, 5557, 5800, 6050,
    6311, 0, 212, 356, 458, 532, 590, 638, 682, 725, 771, 821, 879, 945, 1022,
    1107, 1198, 1290, 1381, 1471, 1561, 1653, 1746, 1842, 1939, 2038, 2139, 2242,
    2348, 2455, 2565, 2678, 2794, 2913, 3035, 3160, 3298, 3445, 3600, 3760, 3933,
    4113, 4301, 4498, 4704, 4918, 5143, 5374, 5617, 5866, 6128, 6404, 0, 220,
    367, 468, 540, 595, 639, 677, 715, 755, 801, 853, 916, 988, 1069, 1158, 1247,
    1335, 1422, 1510, 1601, 1693, 1787, 1883, 1981, 2082, 2185, 2291, 2399, 2510,
    2623, 2740, 2861, 2984, 3112, 3252, 3401, 3562, 3731, 3902, 4092, 4285, 4490,
    4702, 4928, 5157, 5398, 5649, 5907, 6181, 6463, 0, 228, 377, 478, 547, 596,
    636, 669, 701, 737, 776, 824, 881, 949, 1028, 1114, 1198, 1283, 1367, 1453,
    1541, 1632, 1724, 1819, 1916, 2016, 2119, 2224, 2332, 2443, 2558, 2675, 2797,
    2922, 3052, 3195, 3348, 3508, 3681, 3863, 4054, 4256, 4463, 4684, 4915, 5150,
    5404, 5659, 5928, 6211, 6498, 0, 236, 387, 486, 552, 599, 631, 660, 685, 714,
    748, 791, 843, 908, 985, 1067, 1148, 1230, 1312, 1395, 1481, 1570, 1660,
    1754, 1850, 1949, 2051, 2156, 2264, 2375, 2490, 2608, 2731, 2857, 2988, 3130,
    3286, 3451, 3622, 3810, 4005, 4211, 4427, 4655, 4887, 5139, 5389, 5652, 5934,
    6217, 6516, 0, 243, 397, 495, 557, 598, 626, 647, 669, 690, 720, 756, 805,
    866, 940, 1020, 1099, 1177, 1257, 1338, 1422, 1508, 1598, 1690, 1785, 1883,
    1984, 2089, 2196, 2308, 2423, 2542, 2665, 2793, 2925, 3066, 3220, 3387, 3566,
    3754, 3956, 4162, 4387, 4613, 4856, 5106, 5365, 5642, 5917, 6213, 6522, 0,
    251, 406, 503, 562, 597, 620, 635, 649, 665, 688, 720, 765, 823, 895, 972,
    1050, 1125, 1202, 1281, 1363, 1448, 1536, 1626, 1720, 1818, 1918, 2022, 2129,
    2241, 2356, 2475, 2599, 2728, 2862, 3000, 3154, 3321, 3503, 3693, 3896, 4108,
    4333, 4568, 4816, 5068, 5336, 5610, 5899, 6199, 6509, 0, 259, 416, 511, 566,
    596, 613, 623, 629, 642, 656, 685, 725, 782, 853, 927, 1001, 1074, 1148,
    1225, 1305, 1388, 1474, 1563, 1656, 1752, 1852, 1955, 2062, 2173, 2289, 2409,
    2533, 2663, 2797, 2937, 3093, 3261, 3441, 3635, 3837, 4059, 4281, 4522, 4768,
    5027, 5300, 5578, 5869, 6176, 6489, 0, 267, 427, 519, 570, 597, 606, 610,
    609, 614, 626, 650, 686, 742, 811, 883, 954, 1024, 1096, 1171, 1249, 1330,
    1415, 1503, 1595, 1690, 1789, 1892, 1999, 2110, 2226, 2346, 2471, 2602, 2737,
    2879, 3034, 3201, 3381, 3579, 3784, 3999, 4228, 4473, 4721, 4987, 5257, 5540,
    5842, 6142, 6464, 0, 276, 437, 528, 575, 595, 599, 596, 592, 588, 597, 614,
    650, 705, 774, 842, 910, 978, 1048, 1121, 1197, 1277, 1360, 1447, 1537, 1632,
    1730, 1833, 1940, 2051, 2167, 2288, 2414, 2545, 2682, 2824, 2982, 3149, 3329,
    3523, 3733, 3948, 4184, 4424, 4675, 4941, 5214, 5507, 5799, 6112, 6435, 0,
    285, 448, 537, 580, 595, 594, 582, 572, 565, 566, 582, 618, 672, 740, 807,
    873, 939, 1007, 1078, 1153, 1231, 1314, 1400, 1490, 1584, 1682, 1784, 1891,
    2003, 2120, 2241, 2368, 2500, 2638, 2782, 2938, 3105, 3287, 3477, 3684, 3904,
    4134, 4376, 4636, 4898, 5176, 5465, 5764, 6078, 6406, 0, 294, 460, 547, 587,
    595, 587, 572, 555, 543, 538, 553, 587, 643, 711, 777, 841, 905, 972, 1042,
    1116, 1193, 1275, 1360, 1450, 1544, 1642, 1744, 1852, 1964, 2081, 2204, 2332,
    2465, 2605, 2750, 2904, 3070, 3248, 3441, 3645, 3866, 4096, 4340, 4592, 4859,
    5142, 5427, 5728, 6048, 6370, 0, 305, 473, 558, 593, 597, 582, 562, 538, 522,
    515, 528, 563, 620, 688, 753, 816, 879, 945, 1014, 1087, 1164, 1245, 1330,
    1420, 1514, 1612, 1715, 1823, 1936, 2054, 2178, 2307, 2442, 2583, 2731, 2884,
    3048, 3224, 3417, 3618, 3837, 4063, 4308, 4558, 4826, 5106, 5393, 5700, 6014,
    6342, 0, 316, 486, 570, 601, 600, 580, 553, 525, 503, 494, 506, 544, 603,
    671, 735, 797, 859, 924, 993, 1065, 1142, 1223, 1308, 1397, 1492, 1591, 1694,
    1803, 1917, 2037, 2162, 2292, 2429, 2572, 2721, 2876, 3042, 3215, 3403, 3603,
    3816, 4043, 4282, 4533, 4801, 5077, 5367, 5671, 5985, 6314, 0, 327, 501, 583,
    610, 603, 580, 545, 514, 488, 480, 492, 532, 594, 662, 725, 786, 848, 913,
    982, 1054, 1131, 1212, 1298, 1388, 1483, 1583, 1687, 1797, 1913, 2034, 2160,
    2293, 2431, 2576, 2727, 2884, 3049, 3222, 3407, 3603, 3809, 4033, 4266, 4519,
    4779, 5055, 5348, 5647, 5962, 6296, 0, 340, 516, 597, 621, 609, 580, 541,
    505, 478, 468, 483, 527, 591, 660, 724, 785, 847, 912, 981, 1054, 1131, 1213,
    1299, 1390, 1486, 1587, 1693, 1805, 1922, 2044, 2173, 2307, 2448, 2594, 2747,
    2907, 3072, 3244, 3424, 3615, 3818, 4034, 4264, 4511, 4769, 5043, 5331, 5632,
    5948, 6281, 0, 352, 533, 613, 634, 617, 583, 540, 499, 472, 463, 482, 529,
    597, 667, 731, 792, 855, 921, 991, 1065, 1143, 1226, 1313, 1406, 1503, 1606,
    1713, 1827, 1945, 2070, 2200, 2337, 2479, 2628, 2783, 2944, 3110, 3280, 3458,
    3644, 3841, 4050, 4275, 4515, 4770, 5043, 5326, 5626, 5944, 6271, 0, 367,
    552, 631, 647, 628, 588, 543, 500, 472, 464, 487, 538, 609, 682, 747, 809,
    873, 940, 1011, 1087, 1166, 1251, 1340, 1434, 1533, 1637, 1746, 1861, 1982,
    2109, 2241, 2379, 2524, 2675, 2832, 2995, 3160, 3330, 3505, 3690, 3879, 4082,
    4299, 4533, 4783, 5048, 5331, 5629, 5944, 6273, 0, 382, 571, 650, 664, 641,
    596, 547, 504, 477, 473, 498, 554, 628, 702, 768, 833, 898, 966, 1039, 1116,
    1197, 1283, 1374, 1469, 1570, 1676, 1787, 1904, 2026, 2155, 2289, 2429, 2576,
    2728, 2887, 3052, 3220, 3391, 3565, 3745, 3932, 4129, 4339, 4565, 4808, 5069,
    5347, 5644, 5955, 6283, 0, 398, 593, 672, 682, 656, 609, 558, 514, 487, 486,
    515, 573, 652, 728, 796, 862, 929, 999, 1074, 1152, 1235, 1323, 1415, 1512,
    1614, 1722, 1835, 1954, 2078, 2208, 2344, 2486, 2634, 2789, 2949, 3116, 3285,
    3456, 3632, 3813, 3998, 4189, 4392, 4610, 4846, 5101, 5375, 5667, 5977, 6305,
    0, 415, 616, 695, 704, 674, 624, 571, 528, 503, 504, 536, 596, 678, 757, 827,
    895, 964, 1036, 1112, 1192, 1277, 1366, 1460, 1559, 1663, 1772, 1887, 2007,
    2133, 2264, 2402, 2546, 2695, 2851, 3013, 3181, 3352, 3523, 3701, 3882, 4068,
    4259, 4459, 4669, 4898, 5146, 5414, 5703, 6011, 6339, 0, 433, 640, 720, 727,
    695, 644, 590, 547, 523, 525, 559, 622, 707, 789, 862, 932, 1003, 1077, 1154,
    1236, 1322, 1413, 1509, 1609, 1715, 1825, 1941, 2063, 2190, 2323, 2462, 2607,
    2758, 2915, 3078, 3248, 3420, 3593, 3771, 3953, 4140, 4332, 4533, 4739, 4962,
    5205, 5467, 5752, 6057, 6383, 0, 452, 666, 747, 753, 717, 666, 611, 569, 545,
    549, 585, 651, 739, 824, 899, 971, 1043, 1119, 1198, 1282, 1370, 1462, 1559,
    1661, 1768, 1880, 1997, 2120, 2248, 2383, 2523, 2669, 2821, 2980, 3144, 3315,
    3488, 3662, 3842, 4026, 4214, 4408, 4613, 4818, 5041, 5273, 5533, 5812, 6114,
    6439, 0, 471, 693, 775, 781, 743, 689, 635, 594, 570, 575, 612, 681, 772,
    860, 937, 1011, 1085, 1163, 1244, 1329, 1418, 1511, 1610, 1713, 1821, 1934,
    2052, 2176, 2306, 2441, 2583, 2730, 2883, 3043, 3208, 3380, 3555, 3731, 3913,
    4098, 4288, 4484, 4691, 4902, 5122, 5352, 5606, 5885, 6181, 6507, 0, 493,
    721, 805, 809, 772, 717, 662, 620, 597, 602, 642, 713, 806, 896, 976, 1051,
    1127, 1206, 1288, 1375, 1465, 1560, 1660, 1764, 1873, 1987, 2106, 2231, 2362,
    2498, 2641, 2789, 2943, 3103, 3270, 3443, 3619, 3798, 3982, 4170, 4362, 4560,
    4770, 4983, 5207, 5442, 5694, 5967, 6264, 6586, 0, 514, 750, 836, 840, 802,
    746, 690, 647, 625, 631, 672, 745, 841, 933, 1014, 1091, 1169, 1249, 1332,
    1420, 1511, 1607, 1708, 1813, 1923, 2038, 2158, 2284, 2415, 2553, 2696, 2845,
    3000, 3162, 3329, 3504, 3682, 3862, 4049, 4240, 4435, 4636, 4848, 5065, 5293,
    5532, 5787, 6055, 6351, 6674, 0, 536, 780, 869, 872, 833, 776, 719, 676, 653,
    660, 702, 777, 875, 969, 1052, 1131, 1209, 1290, 1375, 1463, 1555, 1652,
    1753, 1859, 1970, 2086, 2207, 2334, 2466, 2604, 2749, 2899, 3055, 3217, 3386,
    3562, 3742, 3925, 4114, 4308, 4507, 4711, 4927, 5146, 5378, 5621, 5880, 6150,
    6449, 6771, 0, 559, 810, 901, 905, 865, 808, 749, 706, 683, 690, 733, 809,
    908, 1004, 1089, 1168, 1248, 1330, 1415, 1504, 1597, 1695, 1797, 1904, 2015,
    2132, 2254, 2381, 2514, 2653, 2799, 2950, 3107, 3271, 3441, 3618, 3800, 3985,
    4177, 4374, 4577, 4785, 5004, 5228, 5463, 5710, 5974, 6250, 6551, 6872, 0,
    580, 841, 934, 939, 898, 839, 780, 736, 712, 719, 763, 840, 941, 1039, 1124,
    1205, 1285, 1367, 1454, 1543, 1637, 1735, 1838, 1945, 2058, 2175, 2298, 2426,
    2560, 2700, 2847, 2999, 3157, 3322, 3494, 3673, 3856, 4044, 4239, 4439, 4645,
    4857, 5080, 5308, 5548, 5799, 6067, 6349, 6652, 6976, 0, 602, 871, 968, 973,
    932, 872, 811, 766, 741, 749, 792, 871, 973, 1072, 1158, 1240, 1321, 1404,
    1490, 1581, 1675, 1774, 1878, 1986, 2099, 2217, 2341, 2470, 2605, 2746, 2893,
    3047, 3207, 3373, 3546, 3726, 3912, 4103, 4300, 4503, 4712, 4929, 5155, 5388,
    5632, 5887, 6159, 6445, 6754, 7083, 0, 623, 901, 1001, 1007, 965, 904, 842,
    796, 771, 777, 821, 901, 1005, 1104, 1192, 1274, 1355, 1439, 1526, 1617,
    1712, 1812, 1916, 2025, 2139, 2258, 2382, 2512, 2649, 2791, 2939, 3094, 3255,
    3423, 3598, 3780, 3967, 4160, 4360, 4566, 4779, 4999, 5230, 5467, 5715, 5975,
    6250, 6540, 6853, 7187, 0, 644, 930, 1034, 1041, 999, 937, 873, 826, 800,
    806, 850, 931, 1036, 1137, 1225, 1308, 1390, 1474, 1562, 1653, 1749, 1849,
    1954, 2063, 2178, 2298, 2424, 2555, 2692, 2836, 2986, 3142, 3304, 3474, 3650,
    3834, 4023, 4218, 4420, 4629, 4845, 5069, 5303, 5545, 5797, 6061, 6340, 6634,
    6951, 7290, 0, 665, 960, 1068, 1076, 1033, 969, 904, 856, 829, 834, 879, 960,
    1067, 1169, 1258, 1341, 1424, 1509, 1597, 1689, 1785, 1886, 1992, 2102, 2218,
    2339, 2466, 2598, 2737, 2881, 3032, 3190, 3354, 3525, 3703, 3888, 4079, 4276,
    4480, 4691, 4911, 5139, 5376, 5622, 5878, 6146, 6429, 6727, 7047, 7390, 0,
    685, 990, 1101, 1110, 1066, 1002, 935, 886, 858, 863, 908, 990, 1098, 1201,
    1291, 1375, 1458, 1543, 1632, 1725, 1822, 1924, 2030, 2142, 2258, 2380, 2508,
    2642, 2782, 2928, 3080, 3239, 3404, 3576, 3756, 3942, 4135, 4334, 4540, 4755,
    4977, 5208, 5449, 5698, 5958, 6230, 6516, 6818, 7142, 7488, 0, 705, 1019,
    1134, 1144, 1100, 1034, 967, 916, 887, 892, 937, 1021, 1130, 1234, 1324,
    1409, 1493, 1579, 1668, 1762, 1860, 1962, 2069, 2182, 2299, 2423, 2552, 2686,
    2827, 2974, 3128, 3288, 3455, 3629, 3809, 3997, 4192, 4393, 4601, 4818, 5043,
    5277, 5521, 5773, 6037, 6312, 6602, 6907, 7235, 7585, 0, 725, 1048, 1167,
    1178, 1133, 1066, 998, 946, 916, 921, 967, 1051, 1162, 1267, 1358, 1444,
    1528, 1615, 1705, 1799, 1898, 2001, 2109, 2222, 2341, 2465, 2596, 2732, 2874,
    3022, 3177, 3338, 3507, 3682, 3864, 4053, 4249, 4452, 4663, 4881, 5109, 5346,
    5592, 5848, 6114, 6392, 6686, 6995, 7326, 7680, 0, 745, 1077, 1200, 1212,
    1167, 1099, 1029, 976, 946, 951, 997, 1083, 1195, 1301, 1393, 1479, 1564,
    1651, 1742, 1837, 1936, 2040, 2149, 2264, 2384, 2509, 2640, 2777, 2921, 3070,
    3227, 3389, 3559, 3735, 3919, 4110, 4307, 4512, 4724, 4945, 5175, 5414, 5663,
    5921, 6190, 6472, 6768, 7081, 7416, 7774, 0, 765, 1106, 1232, 1246, 1200,
    1132, 1061, 1007, 976, 982, 1028, 1115, 1228, 1335, 1428, 1514, 1600, 1688,
    1779, 1875, 1975, 2080, 2190, 2306, 2427, 2553, 2686, 2824, 2969, 3120, 3277,
    3441, 3612, 3790, 3975, 4167, 4366, 4572, 4786, 5009, 5240, 5482, 5733, 5994,
    6265, 6549, 6849, 7165, 7504, 7866, 0, 784, 1135, 1265, 1280, 1234, 1165,
    1093, 1038, 1007, 1013, 1060, 1147, 1261, 1369, 1463, 1550, 1636, 1725, 1817,
    1914, 2015, 2121, 2232, 2348, 2470, 2598, 2732, 2871, 3017, 3170, 3328, 3494,
    3666, 3845, 4032, 4225, 4426, 4633, 4848, 5072, 5306, 5549, 5802, 6065, 6339,
    6626, 6928, 7248, 7591, 7957, 0, 804, 1163, 1298, 1314, 1268, 1198, 1125,
    1070, 1039, 1044, 1092, 1180, 1295, 1404, 1498, 1586, 1673, 1762, 1855, 1952,
    2054, 2161, 2274, 2391, 2514, 2643, 2778, 2919, 3067, 3220, 3381, 3548, 3721,
    3902, 4089, 4284, 4486, 4695, 4911, 5137, 5371, 5615, 5870, 6134, 6411, 6701,
    7007, 7331, 7677, 8048, 0, 823, 1192, 1330, 1348, 1302, 1232, 1158, 1103,
    1071, 1076, 1124, 1212, 1328, 1438, 1533, 1621, 1709, 1799, 1893, 1991, 2094,
    2203, 2316, 2435, 2559, 2690, 2826, 2968, 3117, 3272, 3434, 3602, 3777, 3959,
    4148, 4344, 4547, 4757, 4975, 5201, 5437, 5682, 5937, 6203, 6482, 6775, 7084,
    7412, 7763, 8138, 0, 842, 1220, 1363, 1382, 1337, 1266, 1191, 1135, 1103,
    1107, 1155, 1245, 1361, 1472, 1568, 1657, 1746, 1837, 1932, 2031, 2135, 2244,
    2359, 2479, 2605, 2736, 2874, 3018, 3168, 3325, 3488, 3658, 3834, 4017, 4208,
    4405, 4609, 4821, 5039, 5266, 5502, 5748, 6004, 6271, 6552, 6848, 7161, 7493,
    7848, 8228, 0, 861, 1248, 1396, 1416, 1372, 1300, 1225, 1168, 1135, 1139,
    1187, 1277, 1395, 1506, 1603, 1693, 1782, 1874, 1970, 2070, 2176, 2286, 2402,
    2523, 2651, 2784, 2923, 3069, 3220, 3378, 3543, 3714, 3892, 4077, 4268, 4467,
    4672, 4885, 5104, 5331, 5567, 5813, 6070, 6339, 6622, 6921, 7237, 7574, 7934,
    8319, 0, 880, 1277, 1428, 1451, 1407, 1334, 1258, 1200, 1167, 1171, 1219,
    1309, 1428, 1540, 1638, 1729, 1819, 1912, 2009, 2110, 2217, 2329, 2446, 2569,
    2697, 2832, 2973, 3120, 3273, 3433, 3599, 3772, 3951, 4137, 4330, 4530, 4736,
    4949, 5169, 5397, 5633, 5879, 6137, 6407, 6692, 6994, 7314, 7656, 8020, 8410,
    0, 899, 1305, 1461, 1486, 1442, 1369, 1292, 1233, 1198, 1202, 1250, 1341,
    1460, 1574, 1672, 1764, 1856, 1950, 2048, 2151, 2258, 2372, 2490, 2615, 2745,
    2881, 3024, 3172, 3327, 3488, 3656, 3830, 4011, 4198, 4392, 4593, 4801, 5015,
    5235, 5463, 5699, 5945, 6204, 6475, 6763, 7068, 7392, 7738, 8107, 8502, 0,
    918, 1334, 1494, 1521, 1477, 1403, 1326, 1266, 1230, 1233, 1281, 1372, 1493,
    1607, 1707, 1800, 1893, 1988, 2087, 2191, 2300, 2415, 2535, 2661, 2793, 2931,
    3075, 3225, 3382, 3545, 3714, 3890, 4072, 4261, 4456, 4658, 4867, 5082, 5302,
    5530, 5766, 6013, 6272, 6545, 6835, 7144, 7472, 7822, 8195, 8595, 0, 937,
    1362, 1528, 1556, 1512, 1438, 1359, 1298, 1262, 1264, 1312, 1404, 1525, 1641,
    1741, 1836, 1929, 2026, 2127, 2232, 2343, 2459, 2581, 2708, 2842, 2981, 3127,
    3279, 3437, 3602, 3773, 3950, 4134, 4324, 4521, 4724, 4934, 5149, 5370, 5598,
    5834, 6081, 6341, 6617, 6910, 7221, 7553, 7908, 8286, 8691, 0, 956, 1391,
    1561, 1590, 1547, 1472, 1392, 1330, 1293, 1295, 1342, 1435, 1557, 1674, 1776,
    1871, 1966, 2065, 2167, 2274, 2386, 2503, 2627, 2756, 2891, 3033, 3180, 3334,
    3494, 3660, 3832, 4011, 4196, 4388, 4586, 4791, 5002, 5218, 5439, 5668, 5904,
    6152, 6413, 6691, 6986, 7302, 7637, 7996, 8379, 8788, 0, 975, 1419, 1594,
    1625, 1582, 1506, 1426, 1363, 1324, 1325, 1372, 1465, 1589, 1707, 1810, 1907,
    2004, 2103, 2207, 2315, 2429, 2548, 2674, 2805, 2942, 3085, 3234, 3389, 3551,
    3719, 3893, 4073, 4260, 4453, 4653, 4859, 5070, 5287, 5510, 5739, 5976, 6225,
    6488, 6768, 7066, 7385, 7724, 8088, 8474, 8888, 0, 994, 1448, 1627, 1660,
    1617, 1541, 1459, 1394, 1355, 1355, 1402, 1496, 1620, 1740, 1844, 1943, 2041,
    2142, 2247, 2357, 2473, 2594, 2721, 2854, 2992, 3137, 3288, 3446, 3609, 3779,
    3954, 4136, 4324, 4519, 4720, 4927, 5140, 5358, 5582, 5811, 6050, 6300, 6565,
    6848, 7150, 7472, 7815, 8182, 8573, 8991, 0, 1013, 1476, 1660, 1695, 1652,
    1575, 1491, 1426, 1385, 1385, 1432, 1526, 1651, 1772, 1879, 1979, 2078, 2181,
    2288, 2400, 2517, 2640, 2769, 2903, 3044, 3191, 3343, 3502, 3667, 3839, 4016,
    4199, 4389, 4585, 4787, 4996, 5211, 5430, 5655, 5886, 6126, 6379, 6647, 6932,
    7237, 7562, 7909, 8281, 8675, 9098, 0, 1031, 1504, 1692, 1729, 1686, 1609,
    1524, 1457, 1415, 1414, 1461, 1556, 1683, 1805, 1913, 2015, 2116, 2221, 2329,
    2443, 2562, 2687, 2817, 2954, 3096, 3244, 3399, 3560, 3726, 3899, 4078, 4263,
    4454, 4652, 4856, 5066, 5282, 5503, 5730, 5963, 6206, 6461, 6732, 7020, 7328,
    7657, 8007, 8383, 8781, 9207, 0, 1050, 1533, 1725, 1764, 1721, 1642, 1557,
    1489, 1445, 1444, 1491, 1586, 1714, 1838, 1947, 2051, 2154, 2261, 2371, 2487,
    2607, 2734, 2866, 3004, 3148, 3298, 3455, 3617, 3785, 3960, 4140, 4327, 4520,
    4719, 4924, 5137, 5354, 5577, 5806, 6042, 6288, 6546, 6820, 7112, 7423, 7756,
    8110, 8488, 8890, 9320, 0, 1068, 1561, 1758, 1798, 1755, 1676, 1589, 1520,
    1475, 1473, 1520, 1615, 1745, 1871, 1982, 2088, 2193, 2301, 2413, 2530, 2653,
    2781, 2915, 3055, 3201, 3353, 3511, 3675, 3845, 4021, 4203, 4391, 4585, 4786,
    4994, 5208, 5427, 5652, 5884, 6123, 6372, 6635, 6912, 7208, 7523, 7858, 8216,
    8598, 9003, 9437, 0, 1087, 1588, 1790, 1832, 1789, 1709, 1621, 1551, 1505,
    1502, 1549, 1645, 1776, 1904, 2018, 2125, 2232, 2342, 2456, 2575, 2699, 2829,
    2965, 3106, 3254, 3407, 3567, 3732, 3904, 4081, 4265, 4455, 4651, 4854, 5063,
    5279, 5501, 5729, 5963, 6206, 6459, 6726, 7008, 7308, 7626, 7965, 8326, 8711,
    9120, 9557, 0, 1105, 1616, 1822, 1866, 1823, 1743, 1653, 1581, 1535, 1531,
    1578, 1675, 1808, 1938, 2053, 2162, 2271, 2383, 2498, 2619, 2745, 2877, 3014,
    3157, 3306, 3461, 3622, 3789, 3962, 4142, 4327, 4519, 4717, 4921, 5132, 5351,
    5575, 5806, 6044, 6291, 6549, 6820, 7107, 7411, 7733, 8076, 8439, 8828, 9240,
    9680, 0, 1123, 1644, 1854, 1900, 1857, 1776, 1685, 1612, 1564, 1560, 1607,
    1706, 1840, 1972, 2089, 2200, 2311, 2424, 2541, 2664, 2791, 2924, 3063, 3208,
    3358, 3515, 3677, 3846, 4021, 4202, 4389, 4582, 4782, 4988, 5202, 5423, 5650,
    5884, 6126, 6377, 6640, 6917, 7208, 7517, 7843, 8189, 8556, 8947, 9363, 9806,
    0, 1140, 1671, 1886, 1933, 1891, 1808, 1717, 1643, 1594, 1589, 1637, 1737,
    1873, 2007, 2125, 2238, 2350, 2465, 2584, 2708, 2837, 2972, 3112, 3258, 3410,
    3568, 3732, 3902, 4079, 4261, 4450, 4645, 4847, 5056, 5272, 5495, 5725, 5962,
    6208, 6465, 6733, 7015, 7312, 7625, 7955, 8305, 8676, 9071, 9489, 9934, 0,
    1156, 1697, 1918, 1966, 1924, 1841, 1748, 1673, 1624, 1618, 1666, 1767, 1906,
    2041, 2162, 2276, 2390, 2506, 2627, 2752, 2883, 3019, 3161, 3308, 3462, 3621,
    3787, 3958, 4136, 4320, 4511, 4708, 4912, 5123, 5341, 5567, 5800, 6042, 6292,
    6553, 6827, 7115, 7417, 7735, 8070, 8424, 8798, 9195, 9616, 10062, 0, 1172,
    1723, 1949, 1999, 1956, 1873, 1779, 1703, 1653, 1648, 1696, 1799, 1939, 2076,
    2199, 2315, 2430, 2547, 2669, 2796, 2928, 3065, 3208, 3357, 3512, 3673, 3840,
    4013, 4193, 4379, 4571, 4770, 4976, 5190, 5410, 5639, 5876, 6121, 6376, 6643,
    6922, 7216, 7523, 7846, 8185, 8543, 8921, 9321, 9743, 10191, 0, 1187, 1749,
    1981, 2033, 1990, 1904, 1810, 1733, 1683, 1677, 1726, 1830, 1972, 2111, 2235,
    2353, 2469, 2588, 2711, 2839, 2973, 3111, 3256, 3406, 3562, 3724, 3893, 4068,
    4249, 4437, 4631, 4832, 5041, 5256, 5480, 5712, 5952, 6202, 6461, 6733, 7018,
    7317, 7629, 7957, 8302, 8664, 9044, 9446, 9870, 10319, 0, 1200, 1774, 2013,
    2069, 2024, 1936, 1840, 1763, 1712, 1707, 1756, 1861, 2005, 2146, 2272, 2390,
    2508, 2628, 2753, 2882, 3016, 3156, 3302, 3454, 3611, 3775, 3945, 4121, 4304,
    4494, 4690, 4894, 5105, 5323, 5549, 5784, 6028, 6282, 6546, 6823, 7114, 7418,
    7736, 8069, 8418, 8783, 9166, 9570, 9994, 10445, 0, 1216, 1798, 2043, 2103,
    2057, 1973, 1871, 1792, 1741, 1736, 1786, 1893, 2039, 2181, 2308, 2428, 2546,
    2668, 2794, 2924, 3060, 3201, 3348, 3501, 3660, 3825, 3997, 4175, 4359, 4551,
    4749, 4955, 5168, 5390, 5619, 5857, 6105, 6363, 6632, 6914, 7210, 7519, 7842,
    8180, 8533, 8901, 9287, 9691, 10118, 10569, 0, 1229, 1822, 2074, 2135, 2096,
    2008, 1905, 1822, 1769, 1764, 1816, 1924, 2072, 2216, 2344, 2464, 2584, 2707,
    2834, 2965, 3102, 3244, 3393, 3547, 3707, 3874, 4047, 4227, 4414, 4607, 4808,
    5016, 5232, 5456, 5689, 5931, 6182, 6445, 6718, 7005, 7306, 7620, 7948, 8290,
    8646, 9017, 9404, 9811, 10239, 10691, 0, 1243, 1845, 2104, 2169, 2131, 2041,
    1937, 1851, 1797, 1792, 1845, 1955, 2104, 2250, 2379, 2500, 2621, 2745, 2873,
    3005, 3143, 3287, 3436, 3592, 3754, 3922, 4097, 4279, 4468, 4663, 4866, 5077,
    5296, 5523, 5759, 6005, 6260, 6527, 6805, 7096, 7402, 7720, 8053, 8398, 8757,
    9130, 9518, 9928, 10358, 10810, 0, 1255, 1868, 2133, 2203, 2167, 2077, 1973,
    1882, 1825, 1819, 1873, 1985, 2136, 2283, 2413, 2535, 2657, 2782, 2911, 3044,
    3183, 3328, 3479, 3636, 3800, 3970, 4146, 4330, 4521, 4719, 4925, 5138, 5360,
    5591, 5830, 6080, 6339, 6610, 6892, 7188, 7497, 7820, 8156, 8504, 8865, 9241,
    9632, 10042, 10474, 10931, 0, 1267, 1890, 2163, 2239, 2204, 2111, 2007, 1914,
    1855, 1847, 1899, 2013, 2164, 2313, 2444, 2568, 2691, 2817, 2947, 3082, 3222,
    3368, 3521, 3679, 3844, 4016, 4195, 4381, 4574, 4775, 4983, 5200, 5425, 5659,
    5902, 6155, 6419, 6693, 6980, 7279, 7592, 7918, 8256, 8607, 8969, 9347, 9742,
    10152, 10587, 11046, 0, 1279, 1913, 2193, 2272, 2240, 2151, 2046, 1947, 1886,
    1872, 1922, 2035, 2188, 2343, 2475, 2599, 2724, 2850, 2981, 3118, 3259, 3407,
    3561, 3721, 3888, 4062, 4243, 4431, 4627, 4830, 5042, 5261, 5490, 5727, 5974,
    6231, 6498, 6776, 7066, 7370, 7685, 8014, 8354, 8706, 9071, 9452, 9848,
    10263, 10698, 11158, 0, 1290, 1936, 2224, 2311, 2278, 2190, 2081, 1981, 1917,
    1900, 1946, 2057, 2212, 2366, 2500, 2627, 2752, 2881, 3013, 3151, 3294, 3444,
    3599, 3762, 3931, 4107, 4290, 4481, 4680, 4886, 5100, 5323, 5554, 5795, 6046,
    6306, 6577, 6859, 7152, 7458, 7777, 8107, 8449, 8803, 9171, 9551, 9951,
    10369, 10804, 11270, 0, 1304, 1959, 2256, 2346, 2317, 2233, 2121, 2020, 1950,
    1928, 1970, 2075, 2229, 2388, 2525, 2654, 2781, 2911, 3045, 3184, 3329, 3481,
    3638, 3803, 3974, 4152, 4338, 4531, 4732, 4941, 5159, 5384, 5619, 5863, 6117,
    6381, 6655, 6940, 7237, 7545, 7866, 8198, 8542, 8898, 9268, 9652, 10052,
    10472, 10913, 11375, 0, 1316, 1982, 2289, 2383, 2359, 2276, 2162, 2063, 1985,
    1960, 1995, 2099, 2251, 2410, 2550, 2681, 2810, 2942, 3078, 3219, 3366, 3519,
    3679, 3845, 4019, 4199, 4387, 4583, 4787, 4998, 5218, 5446, 5684, 5931, 6188,
    6456, 6733, 7021, 7320, 7631, 7954, 8287, 8633, 8990, 9362, 9748, 10149,
    10573, 11017, 11481, 0, 1328, 2006, 2321, 2423, 2405, 2318, 2210, 2105, 2025,
    1993, 2023, 2123, 2270, 2436, 2579, 2712, 2843, 2977, 3115, 3258, 3407, 3562,
    3723, 3892, 4067, 4250, 4440, 4638, 4844, 5057, 5280, 5510, 5750, 6000, 6260,
    6530, 6809, 7101, 7403, 7716, 8040, 8375, 8722, 9081, 9453, 9842, 10246,
    10670, 11117, 11587, 0, 1339, 2030, 2354, 2465, 2449, 2365, 2260, 2149, 2070,
    2031, 2056, 2149, 2297, 2461, 2609, 2746, 2880, 3016, 3155, 3300, 3451, 3608,
    3772, 3942, 4119, 4304, 4496, 4696, 4903, 5119, 5343, 5576, 5818, 6070, 6332,
    6605, 6887, 7180, 7484, 7800, 8125, 8462, 8809, 9171, 9544, 9933, 10341,
    10765, 11213, 11685, 0, 1351, 2055, 2389, 2509, 2496, 2416, 2310, 2201, 2117,
    2073, 2092, 2178, 2324, 2493, 2646, 2786, 2922, 3060, 3202, 3349, 3502, 3660,
    3826, 3998, 4177, 4363, 4557, 4758, 4968, 5185, 5411, 5646, 5889, 6143, 6407,
    6681, 6965, 7259, 7565, 7883, 8209, 8546, 8895, 9257, 9633, 10022, 10432,
    10861, 11307, 11784, 0, 1365, 2080, 2425, 2551, 2546, 2475, 2362, 2254, 2169,
    2122, 2133, 2212, 2354, 2527, 2687, 2831, 2970, 3110, 3254, 3403, 3558, 3719,
    3886, 4059, 4240, 4428, 4623, 4826, 5036, 5255, 5482, 5718, 5963, 6218, 6483,
    6758, 7042, 7338, 7647, 7964, 8292, 8631, 8980, 9341, 9719, 10111, 10521,
    10951, 11401, 11875, 0, 1378, 2107, 2464, 2595, 2599, 2526, 2423, 2318, 2224,
    2178, 2181, 2254, 2391, 2565, 2734, 2882, 3024, 3167, 3314, 3465, 3621, 3784,
    3953, 4128, 4310, 4499, 4695, 4899, 5110, 5330, 5557, 5794, 6040, 6295, 6561,
    6836, 7122, 7419, 7727, 8045, 8375, 8712, 9063, 9426, 9803, 10197, 10606,
    11039, 11490, 11967, 0, 1392, 2133, 2501, 2644, 2656, 2586, 2487, 2375, 2286,
    2236, 2237, 2302, 2435, 2610, 2785, 2939, 3086, 3233, 3382, 3535, 3694, 3858,
    4028, 4205, 4388, 4577, 4774, 4978, 5190, 5410, 5638, 5875, 6120, 6376, 6642,
    6917, 7203, 7500, 7808, 8127, 8456, 8795, 9144, 9508, 9884, 10280, 10692,
    11121, 11578, 12055, 0, 1405, 2162, 2541, 2695, 2710, 2648, 2554, 2444, 2361,
    2299, 2297, 2356, 2484, 2660, 2844, 3005, 3157, 3307, 3459, 3615, 3775, 3941,
    4113, 4290, 4474, 4664, 4861, 5066, 5277, 5497, 5724, 5960, 6206, 6460, 6725,
    7000, 7285, 7582, 7890, 8209, 8536, 8876, 9225, 9589, 9967, 10360, 10773,
    11206, 11659, 12140, 0, 1419, 2191, 2582, 2747, 2770, 2718, 2621, 2517, 2426,
    2373, 2364, 2419, 2542, 2720, 2909, 3078, 3236, 3390, 3546, 3704, 3867, 4034,
    4206, 4385, 4569, 4759, 4957, 5161, 5372, 5590, 5817, 6052, 6296, 6549, 6813,
    7086, 7370, 7666, 7973, 8290, 8618, 8956, 9305, 9667, 10045, 10438, 10853,
    11284, 11739, 12219, 0, 1435, 2221, 2628, 2799, 2834, 2786, 2694, 2600, 2507,
    2455, 2441, 2490, 2607, 2784, 2982, 3161, 3325, 3483, 3642, 3803, 3967, 4136,
    4310, 4489, 4673, 4864, 5060, 5263, 5474, 5691, 5916, 6149, 6391, 6643, 6904,
    7175, 7457, 7751, 8057, 8372, 8699, 9034, 9384, 9745, 10122, 10516, 10925,
    11362, 11816, 12294, 0, 1451, 2252, 2671, 2856, 2904, 2856, 2777, 2677, 2591,
    2535, 2524, 2568, 2681, 2856, 3064, 3254, 3424, 3587, 3749, 3913, 4080, 4250,
    4424, 4604, 4788, 4978, 5174, 5376, 5584, 5800, 6023, 6253, 6493, 6741, 6999,
    7268, 7547, 7839, 8142, 8455, 8779, 9114, 9460, 9822, 10195, 10589, 11002,
    11431, 11889, 12369, 0, 1466, 2285, 2718, 2918, 2966, 2937, 2858, 2764, 2686,
    2630, 2617, 2659, 2767, 2939, 3152, 3356, 3535, 3704, 3871, 4037, 4206, 4378,
    4553, 4732, 4916, 5105, 5300, 5500, 5706, 5918, 6138, 6365, 6601, 6845, 7100,
    7365, 7641, 7929, 8228, 8539, 8860, 9192, 9537, 9895, 10270, 10659, 11070,
    11502, 11955, 12437, 0, 1484, 2320, 2769, 2979, 3042, 3022, 2945, 2862, 2780,
    2731, 2714, 2760, 2863, 3034, 3251, 3468, 3659, 3835, 4007, 4177, 4348, 4521,
    4697, 4877, 5060, 5247, 5439, 5636, 5839, 6048, 6264, 6487, 6718, 6957, 7207,
    7466, 7738, 8022, 8317, 8624, 8941, 9270, 9612, 9966, 10339, 10728, 11136,
    11567, 12019, 12499, 0, 1503, 2356, 2821, 3044, 3120, 3102, 3040, 2959, 2885,
    2835, 2825, 2865, 2972, 3140, 3361, 3592, 3794, 3979, 4156, 4330, 4504, 4678,
    4855, 5033, 5215, 5401, 5590, 5784, 5983, 6188, 6399, 6617, 6842, 7075, 7319,
    7574, 7840, 8118, 8408, 8710, 9023, 9348, 9685, 10038, 10406, 10793, 11199,
    11628, 12080, 12556, 0, 1521, 2393, 2874, 3111, 3199, 3193, 3144, 3066, 2999,
    2952, 2949, 2988, 3091, 3261, 3484, 3728, 3945, 4138, 4321, 4499, 4675, 4851,
    5028, 5206, 5386, 5569, 5755, 5945, 6140, 6340, 6545, 6756, 6975, 7202, 7439,
    7687, 7946, 8219, 8503, 8798, 9105, 9425, 9758, 10107, 10472, 10856, 11259,
    11684, 12134, 12611, 0, 1541, 2433, 2931, 3186, 3282, 3292, 3242, 3180, 3117,
    3077, 3071, 3120, 3225, 3395, 3623, 3874, 4111, 4314, 4504, 4686, 4865, 5042,
    5219, 5396, 5574, 5754, 5936, 6122, 6311, 6504, 6703, 6907, 7118, 7339, 7567,
    7807, 8059, 8323, 8599, 8888, 9189, 9503, 9831, 10174, 10535, 10914, 11315,
    11738, 12184, 12659, 0, 1563, 2475, 2992, 3260, 3369, 3391, 3355, 3305, 3242,
    3214, 3210, 3262, 3373, 3545, 3776, 4036, 4293, 4509, 4706, 4894, 5075, 5254,
    5430, 5606, 5781, 5958, 6135, 6315, 6498, 6684, 6875, 7071, 7272, 7485, 7704,
    7935, 8178, 8432, 8700, 8981, 9274, 9581, 9903, 10241, 10596, 10971, 11367,
    11786, 12230, 12700, 0, 1587, 2519, 3054, 3338, 3469, 3495, 3477, 3423, 3382,
    3352, 3366, 3420, 3535, 3712, 3948, 4221, 4488, 4720, 4929, 5123, 5309, 5489,
    5665, 5839, 6012, 6183, 6356, 6529, 6704, 6882, 7064, 7250, 7441, 7641, 7850,
    8071, 8303, 8547, 8806, 9076, 9361, 9659, 9975, 10305, 10655, 11024, 11416,
    11830, 12270, 12737, 0, 1610, 2565, 3120, 3424, 3565, 3609, 3599, 3562, 3528,
    3510, 3527, 3593, 3712, 3895, 4139, 4419, 4708, 4959, 5176, 5375, 5564, 5745,
    5921, 6092, 6260, 6427, 6593, 6758, 6925, 7093, 7265, 7440, 7621, 7811, 8008,
    8216, 8436, 8670, 8916, 9176, 9451, 9740, 10046, 10369, 10712, 11075, 11460,
    11870, 12305, 12767, 0, 1635, 2615, 3190, 3512, 3670, 3732, 3732, 3713, 3681,
    3683, 3702, 3777, 3907, 4098, 4348, 4638, 4942, 5217, 5446, 5654, 5846, 6029,
    6204, 6372, 6536, 6697, 6855, 7012, 7169, 7326, 7486, 7649, 7818, 7994, 8177,
    8372, 8579, 8798, 9031, 9281, 9542, 9823, 10117, 10433, 10766, 11123, 11501,
    11904, 12334, 12792, 0, 1662, 2667, 3265, 3604, 3784, 3856, 3875, 3858, 3848,
    3853, 3894, 3978, 4119, 4320, 4580, 4888, 5199, 5496, 5741, 5958, 6156, 6340,
    6514, 6680, 6839, 6992, 7142, 7289, 7435, 7581, 7728, 7877, 8031, 8191, 8359,
    8539, 8730, 8934, 9153, 9387, 9637, 9903, 10191, 10494, 10821, 11167, 11539,
    11935, 12359, 12811, 0, 1692, 2721, 3343, 3704, 3899, 3988, 4023, 4026, 4029,
    4047, 4099, 4197, 4351, 4563, 4835, 5158, 5496, 5810, 6069, 6295, 6498, 6684,
    6856, 7018, 7170, 7316, 7456, 7592, 7725, 7858, 7990, 8124, 8261, 8404, 8556,
    8718, 8892, 9080, 9282, 9501, 9737, 9990, 10263, 10555, 10871, 11209, 11572,
    11962, 12376, 12822, 0, 1722, 2780, 3426, 3810, 4023, 4135, 4180, 4205, 4221,
    4258, 4322, 4435, 4602, 4832, 5118, 5451, 5809, 6146, 6430, 6666, 6874, 7062,
    7232, 7389, 7535, 7671, 7800, 7923, 8043, 8160, 8276, 8392, 8511, 8635, 8768,
    8910, 9066, 9235, 9420, 9622, 9840, 10080, 10336, 10619, 10921, 11251, 11602,
    11983, 12390, 12827, 0, 1754, 2842, 3514, 3920, 4156, 4284, 4353, 4389, 4427,
    4475, 4562, 4691, 4875, 5121, 5427, 5783, 6144, 6512, 6825, 7073, 7287, 7476,
    7644, 7796, 7933, 8059, 8176, 8285, 8389, 8489, 8587, 8683, 8781, 8885, 8997,
    9118, 9253, 9400, 9566, 9746, 9949, 10170, 10414, 10680, 10971, 11287, 11631,
    11998, 12400, 12828, 0, 1790, 2908, 3607, 4038, 4296, 4447, 4535, 4591, 4646,
    4715, 4820, 4970, 5175, 5437, 5765, 6143, 6535, 6923, 7248, 7510, 7733, 7924,
    8091, 8237, 8366, 8481, 8584, 8678, 8764, 8846, 8925, 9000, 9075, 9156, 9244,
    9342, 9453, 9579, 9720, 9881, 10063, 10266, 10494, 10744, 11020, 11322,
    11655, 12012, 12399, 12823, 0, 1827, 2977, 3706, 4164, 4445, 4620, 4728,
    4807, 4881, 4974, 5100, 5272, 5501, 5787, 6133, 6534, 6956, 7365, 7724, 8012,
    8240, 8432, 8595, 8733, 8851, 8952, 9038, 9114, 9181, 9241, 9294, 9344, 9394,
    9449, 9510, 9583, 9669, 9769, 9887, 10025, 10183, 10368, 10575, 10810, 11068,
    11357, 11671, 12023, 12398, 12809 },

  /* Pooled Parameter (Expression: -power_loss_limit.*ones(51, 4))
   * Referenced by:
   *   '<S18>/Bias1'
   *   '<S18>/Bias2'
   */
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    -23200, -23200, -23200, -23200, -23200, -23200, -23200, -23200, -23200,
    -23200, -23200, -23200, -23200, -23200, -23200, -23200, -23200, -23200,
    -23200, -23200, -23200, -23200, -23200, -23200, -23200, -23200, -23200,
    -23200, -23200, -23200, -23200, -23200, -23200, -23200, -23200, -23200,
    -23200, -23200, -23200, -23200, -23200, -23200, -23200, -23200, -23200,
    -23200, -23200, -23200, -23200, -23200, -23200, -23200, -23200, -23200,
    -23200, -23200, -23200, -23200, -23200, -23200, -23200, -23200, -23200,
    -23200, -23200, -23200, -23200, -23200, -23200, -23200, -23200, -23200,
    -23200, -23200, -23200, -23200, -23200, -23200, -23200, -23200, -23200,
    -23200, -23200, -23200, -23200, -23200, -23200, -23200, -23200, -23200,
    -23200, -23200, -23200, -23200, -23200, -23200, -23200, -23200, -23200,
    -23200, -23200, -23200 },

  /* Computed Parameter: uDLookupTable_bp01Data
   * Referenced by: '<S8>/1-D Lookup Table'
   */
  { 0, 13466, 13498, 13534, 13571, 13607, 13645, 13683, 13721, 13760, 13800,
    13840, 13881, 13923, 13965, 14008, 14052, 14097, 14143, 14189, 14236, 14284,
    14333, 14382, 14433, 14485, 14537, 14591, 14646, 14701, 14758, 14816, 14875,
    14935, 14996, 15058, 15122, 15187, 15253, 15320, 15388, 15530, 15602, 15676,
    15751, 15828, 15906, 15986, 16067, 16149, 16233, 16319, 16406, 16495, 16585,
    16678, 16771, 16867, 16964, 17063, 17163, 17266, 17370, 17475, 17583, 17693,
    17804, 18215 },

  /* Computed Parameter: uDLookupTable_tableData
   * Referenced by: '<S10>/1-D Lookup Table'
   */
  { 12, 12, 12, 15, 10, 11, 7, 7, 7, 7 },

  /* Pooled Parameter (Expression: torque_sweep)
   * Referenced by:
   *   '<S18>/Constant7'
   *   '<S18>/Constant8'
   */
  { 0U, 1024U, 2048U, 3072U, 4096U, 5120U, 6144U, 7168U, 8192U, 9216U, 10240U,
    11264U, 12288U, 13312U, 14336U, 15360U, 16384U, 17408U, 18432U, 19456U,
    20480U, 21504U, 22528U, 23552U, 24576U, 25600U, 26624U, 27648U, 28672U,
    29696U, 30720U, 31744U, 32768U, 33792U, 34816U, 35840U, 36864U, 37888U,
    38912U, 39936U, 40960U, 41984U, 43008U, 44032U, 45056U, 46080U, 47104U,
    48128U, 49152U, 50176U, 51200U },

  /* Computed Parameter: uDLookupTable_tableData_i
   * Referenced by: '<S8>/1-D Lookup Table'
   */
  { 51200U, 51200U, 50522U, 49777U, 49027U, 48273U, 47518U, 46756U, 45994U,
    45228U, 44458U, 43686U, 42912U, 42136U, 41357U, 40577U, 39795U, 39010U,
    38226U, 37439U, 36653U, 35865U, 35076U, 34288U, 33497U, 32709U, 31920U,
    31130U, 30341U, 29555U, 28768U, 27982U, 27197U, 26413U, 25633U, 24852U,
    24074U, 23298U, 22526U, 21754U, 20986U, 19458U, 18700U, 17945U, 17193U,
    16443U, 15700U, 14961U, 14225U, 13494U, 12767U, 12046U, 11330U, 10619U,
    9912U, 9214U, 8520U, 7832U, 7150U, 6474U, 5804U, 5143U, 4487U, 3840U, 3199U,
    2564U, 1939U, 0U },

  /* Pooled Parameter (Expression: [1:51])
   * Referenced by:
   *   '<S18>/Constant3'
   *   '<S18>/Constant5'
   */
  { 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U, 12U, 13U, 14U, 15U, 16U, 17U,
    18U, 19U, 20U, 21U, 22U, 23U, 24U, 25U, 26U, 27U, 28U, 29U, 30U, 31U, 32U,
    33U, 34U, 35U, 36U, 37U, 38U, 39U, 40U, 41U, 42U, 43U, 44U, 45U, 46U, 47U,
    48U, 49U, 50U, 51U }
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
