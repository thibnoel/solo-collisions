/**************************************************
 This code was generated ! 
See the reference example under inference_neural_net_codegen.cpp
**************************************************/

#include <stdlib.h> 
#include <iostream> 
#include <fstream>  
#include "inference_neural_net.hpp"
#include <chrono> 
#include <sstream>
using namespace std::chrono;

int main()
{
    Eigen::Matrix<double, 48, 6> w0;
    Eigen::Matrix<double, 48, 1> b0;
    Eigen::Matrix<double, 18, 48> w1;
    Eigen::Matrix<double, 18, 1> b1;
    Eigen::Matrix<double, 1, 18> w2;
    Eigen::Matrix<double, 1, 1> b2;


    w0 << 0.76384646, 1.4289376, 3.0371968e-05, 1.993529, -0.4594072, 0.025207333, 
      -0.70108646, -0.5648716, -0.64483726, 0.27567902, -0.402105, -0.0933091, 
      0.5716881, 0.45172366, -0.23697372, 0.010339513, 1.1581696, -0.478148, 
      0.5483862, 0.6144341, 0.09768263, -0.9563414, -0.9030756, -0.028955469, 
      0.05136928, 0.65545756, 0.19277573, -0.042392585, 1.3449814, 0.91493064, 
      0.2680345, 0.64817584, -0.41321418, -0.51460654, 1.6535125, 0.5193212, 
      -0.5765667, -0.10136338, 0.21207976, -0.08521718, -1.5645514, 0.21101435, 
      -0.59256935, 0.35978782, -0.17945643, 0.40882182, -0.25057486, -0.58241147, 
      -0.08748391, 0.8763233, -0.1458883, -0.9437897, 0.26132607, -0.073717855, 
      0.64969015, -0.21875955, 0.23711412, 0.3453663, -0.7308029, -0.62907887, 
      -0.34856835, 0.15201809, -0.7927609, 0.26104915, 0.35339984, 0.26755396, 
      -0.72576493, 0.08332373, 0.4313864, -1.2341367, 0.33474937, -0.17430945, 
      -0.1545827, 0.51682264, 0.70872384, 0.0011073344, -0.5045569, 1.0945277, 
      -1.2254959, -0.16826876, 0.33903137, -0.84890306, 0.98450136, -0.27171633, 
      0.40860683, -0.36958188, 0.8105651, -0.13792269, 0.58349323, -0.0854723, 
      0.34104183, 0.41096613, -0.1612357, -0.826845, 1.0116726, 0.89651203, 
      -0.93234223, 0.031131063, 0.4636053, -0.15224606, -0.010739895, 0.6576586, 
      0.6256084, -0.5959441, -0.23276678, 0.37529287, 1.0574031, 0.17253737, 
      0.32253447, -0.44539592, -0.6810228, 0.014765832, 0.1787956, -1.3175409, 
      0.51759315, -1.1502172, -0.047156125, 0.29714575, 0.81693465, -0.8253992, 
      -0.40641126, 0.6517693, 0.78597254, 0.2127377, -0.3783696, 0.11008535, 
      0.37392128, 0.772561, 0.10415375, 1.603865, -0.07686862, -0.051070634, 
      -0.7647166, 0.86615, -0.014881794, 1.4164062, -0.25415462, 0.20358852, 
      -0.14466296, -0.3284479, 0.5344888, 0.76433593, 0.06764936, 0.69301254, 
      -0.22743486, 0.67903274, -0.11720682, -1.0108898, 0.40272826, 0.46534216, 
      0.36055228, 1.3409708, -0.32270652, -0.932932, -0.39893854, 0.442687, 
      0.95596075, 1.12546, -0.060022824, -1.4134107, 0.4817632, -0.25790748, 
      -0.6365651, 0.3927727, -0.118473485, 0.6082654, -0.19370434, 0.02843311, 
      0.7173122, -0.7385539, 0.29534775, 0.20944229, 0.612345, 0.23691943, 
      -0.6496142, 1.095143, 0.18557587, -1.2641876, -0.32781672, -0.08709407, 
      -0.12427234, 0.70985204, -0.58391625, 0.13646579, 0.43982425, 0.26640707, 
      -0.6564447, 0.22328204, -0.81001705, -0.24191824, -0.3734474, 0.6681384, 
      0.63539714, 0.10925316, -0.86195123, 0.12364294, -0.5225151, -0.062005054, 
      0.5642651, 0.97160083, 0.24745345, -1.0625321, 0.05402445, 0.4050547, 
      0.6788225, 1.1085153, 0.0035259738, 1.6976578, -0.21542698, -0.042007305, 
      0.544492, 2.037812, -0.2911711, -0.6976618, 0.6590892, -0.78299475, 
      -0.21646518, -1.3614073, -0.06837659, 0.35655105, -0.60915184, -1.2564502, 
      0.7319507, 1.1560473, -0.13268712, 0.5030044, 0.5068514, -0.18314573, 
      -0.0328226, 0.6501008, 0.013954664, 1.324413, 0.32671404, -0.10764819, 
      -0.72079206, 0.17489205, 0.30837515, 0.44031584, 0.62136847, -0.5969091, 
      0.2531452, 0.73526806, 0.38681075, -1.3362702, 1.1876086, -0.79827166, 
      0.6553497, -0.10093056, -0.27885064, -0.29432765, -0.5719981, 0.29592943, 
      0.11523655, -0.3823854, 1.0209401, 0.41682106, -0.7600483, -0.20218979, 
      -0.7181924, -0.8030316, 0.017442087, 0.9093562, 1.7353997, 0.028773459, 
      0.70625734, 1.1791457, -0.5931409, 0.26022822, 0.18291284, -0.038061377, 
      -0.22879821, 0.28393367, 0.11875486, 0.19348983, -1.7589654, 0.3344477, 
      0.23437315, 0.23233469, 0.4818819, -0.64068556, 0.10309477, -0.3061946, 
      0.7314081, 0.6684328, -0.035085846, -0.044036318, -0.21600783, -0.2872766; 
    b0 << 0.6323151, 
      0.20730999, 
      -0.48776996, 
      -0.37767372, 
      -0.06650461, 
      -0.24082115, 
      -0.3136793, 
      -0.21656014, 
      0.80049735, 
      0.23256409, 
      -0.1357415, 
      0.60887057, 
      -0.4287994, 
      -0.9421118, 
      0.30086422, 
      -0.63990766, 
      0.13853447, 
      0.22765778, 
      0.15120885, 
      0.12873223, 
      -0.043895606, 
      -0.3243116, 
      -0.8884955, 
      0.5896377, 
      -0.2290037, 
      -0.06178913, 
      -0.7772886, 
      0.6204731, 
      0.34120432, 
      0.9690256, 
      -0.24775806, 
      0.009605185, 
      0.42361063, 
      -0.83657753, 
      0.9318798, 
      0.30063367, 
      -0.37228668, 
      -0.19055368, 
      0.46640214, 
      -0.710386, 
      -0.11481006, 
      -0.2628528, 
      -0.1132007, 
      0.80834687, 
      0.20976621, 
      0.690618, 
      0.44300088, 
      0.43889457; 

    w1 << 0.009329814, -0.0852919, 0.25181928, -0.18793277, 0.10733393, 0.39069512, -0.04294214, 0.2700044, -0.44796234, -0.21740673, 0.14958844, -0.15227735, -0.46152276, 0.30643943, -0.114741616, 0.2386441, 0.08335223, 0.062380005, -0.30035058, -0.165302, 0.4283321, 0.20083183, -0.20304248, -0.122723065, 0.16795869, -0.41158378, 0.21037592, -0.09282929, -0.08944143, -0.28693447, 0.051410433, 0.34055907, -0.042332683, 0.19742921, 0.24699643, 0.07758701, 0.41267, -0.33382493, 0.04317595, 0.19034575, -0.018335206, 0.032189406, 0.35388643, 0.13317396, -0.2521341, -0.116753526, -0.24508283, -0.41149145, 
      -0.41691405, 0.1111642, 0.17430134, 0.20309405, -0.037556056, 0.17254922, 0.34234658, 0.08374698, -0.1745266, 0.12116565, 0.50940746, -0.07159028, 0.1362873, 0.16925566, -0.37706015, -0.22989906, -0.18393566, -0.16969623, 0.4942579, -0.12401447, -0.081588715, -0.20618093, -0.31188604, -0.31684276, -0.1249001, -0.12755008, 0.5876124, 0.5611462, 0.018881278, -0.40971327, -0.017675333, -0.51675093, 0.11959465, 0.121478595, -0.0680038, 0.026420336, -0.020639615, -0.45686725, 0.22070064, 0.17652279, 0.16503334, -0.073677264, -0.27814335, 0.14569978, 0.016283158, -0.15141994, -0.23541485, 0.28953317, 
      0.38759547, -0.2655697, -0.22208273, -0.6442123, 0.3550497, -0.13076882, -0.3713544, -0.29976732, 0.27657244, 0.2625828, -0.049736507, -0.08135358, -0.477779, -0.16692175, -0.05825325, 0.16179137, 0.11606392, 0.07924265, 0.1811641, -0.2573821, -0.093424514, 0.15282841, 0.37663525, 0.38306046, -0.33938137, -0.10028324, -0.68899083, -0.31451178, 0.2705668, 0.22962843, 0.07503346, -0.2511089, -0.013438328, -0.43350348, 0.049670402, 0.06450821, -0.5079284, 0.21119139, 0.2365376, 0.13726884, 0.04609934, 0.0051524183, 0.057261877, -0.38375553, 0.42165545, -0.27794406, -0.093754135, -0.074171074, 
      0.07774678, 0.054560628, 0.21187589, -0.14913379, -0.10072, -0.33417562, -0.039407965, 0.027862268, 0.041329198, -0.32738814, 0.0947264, -0.14681798, 0.035873976, 0.13817805, -0.1590732, 0.16036838, 0.36597112, -0.061268315, 0.38721228, -0.30779073, -0.1259745, -0.0668575, 0.19908516, -0.254993, -0.03384715, -0.48742586, -0.4533098, 0.117844746, 0.048079293, 0.27585724, -0.4204763, 0.04033165, -0.2935133, -0.20707506, 0.27103937, -0.05068943, 0.13441154, 0.1344193, 0.5561468, 0.243339, -0.15661621, 0.43471044, -0.01148268, 0.49521697, -0.3988009, 0.05334959, -0.06722847, -0.2337388, 
      -0.5370685, 0.6996183, 0.36318085, 0.046189267, -0.33105183, -0.052325316, 0.43019864, 0.3972172, -0.06782788, 0.14292966, -0.13290924, -0.85881186, -0.8218322, 0.013882624, -0.433031, 0.29706272, -0.52839935, -0.33327267, -0.6953082, -0.6172352, 0.448376, -0.15921575, -0.2233846, 0.76286197, 0.38556615, 0.52660793, -0.6028413, -0.4356149, 0.4779912, -0.01651454, 0.8475698, -0.3098162, -0.060101155, -0.5719686, -0.13192432, -0.05735513, 0.2906412, -0.2703163, 0.63912773, -0.43337148, -0.061779965, -0.18322754, 0.5227334, 0.464376, -0.2020855, 0.65894514, 0.5645941, -0.4776556, 
      -0.2080419, 0.038077205, -0.04898305, 0.5518205, -0.2893756, 0.52660406, 0.39095905, 0.18382318, -0.13748975, 0.11102222, 0.05018626, 0.28129902, -0.2008613, 0.025897121, -0.0806561, -0.1786373, 0.19246249, 0.07562496, -0.07302544, 0.053722583, 0.32698134, -0.30419156, 0.07056857, -0.24636675, 0.115371644, 0.20023167, 0.46478805, 0.003856946, -0.25248128, -0.06115508, 0.013969233, 0.4827737, 0.09686398, 0.45465788, 0.58998346, 0.012731891, 0.26658952, -0.109005824, -0.16427836, 0.14700955, 0.14414792, 0.029944293, 0.008017043, 0.09686398, -0.24683154, -0.009928992, 0.06850245, 0.007005205, 
      0.06336723, 0.009917877, -0.044999182, 0.16316468, 0.044964205, 0.051273994, -0.141183, -0.25294492, 0.16945373, 0.38838744, -0.097156085, -0.393507, -0.11392853, -0.026489811, -0.05888017, -0.10576946, 0.118012995, 0.15762898, 0.2388226, 0.11431901, 0.136463, -0.18439287, 0.43006894, -0.4563525, -0.33746925, -0.24128594, -0.07657044, -0.39503103, -0.074953005, 0.22609198, 0.119098976, 0.50333786, 0.30417696, -0.19344212, -0.05953046, -0.08291183, -0.23590212, 0.31542867, -0.48528352, 0.13133198, -0.30160606, 0.25674096, 0.09777946, -0.14684425, -0.12877211, 0.17674999, -0.18693441, 0.1517745, 
      0.10830815, -0.026509471, -0.2838471, 0.1962758, -0.03743172, -0.070172586, -0.080575086, -0.24032308, 0.38761303, 0.16204107, 0.0760602, -0.3046371, 0.19494474, 0.14583783, 0.096884444, 0.09902602, 0.03519318, -0.17416987, 0.10205458, -0.02724956, -0.14586994, 0.25103283, 0.66658795, -0.034149013, -0.24738765, 0.19949529, -0.42194405, -0.088900685, 0.15268865, 0.1489004, 0.21528748, 0.06176636, -0.23351693, -0.19096261, -0.45149505, -0.34285492, 0.1890021, 0.40886727, -0.29313177, -0.20977895, -0.114763774, 0.34745285, 0.37238792, -0.018051567, 0.053340662, 0.04422989, 0.079588994, 0.080985986, 
      0.03781612, 0.33416262, 0.033740975, -0.03147654, 0.3651105, -0.029480932, -0.21077956, 0.056068003, -0.26870602, 0.49790055, -0.09105943, 0.075288065, -0.72257906, -0.11765386, 0.16313758, 0.14134598, -0.19562614, 0.11606233, -0.41525015, -0.2610264, -0.029562362, 0.09929888, -0.6873646, 0.19203521, 0.20662148, 0.38229233, -0.015470078, 0.4492186, 0.70933247, -0.039543796, -0.31337935, -0.23578551, 0.22616094, 0.18078104, -1.8540252, -0.0033281788, -0.0023595465, -0.09190126, -0.6766951, -0.4616054, -0.13888791, -0.1421114, 0.010819753, -0.4470629, -0.11753979, 0.29698995, -0.0410833, 0.49566266, 
      -0.8873762, 0.121872686, -0.023396654, 0.3743369, -0.18444784, 0.41419083, 0.26614904, -0.25142598, -0.1751067, 0.06540252, 0.07835503, 0.12346002, -0.08016989, 0.038213756, -0.24560773, -0.117926, 0.11000728, -0.23780186, 0.044640843, -0.037224922, 0.27653643, -0.34445322, -0.47416714, 0.26279563, -0.053176574, 0.1338489, 0.331936, 0.41188976, 0.08452789, -0.3070516, 0.08039902, -0.30855447, 0.30175498, 0.30972725, 0.18273449, -0.19912937, -0.025762258, -0.14658277, 0.23768872, -0.13613327, 0.1713796, -0.1161292, 0.1893508, -0.30860627, 0.17755401, 0.11070144, -0.122976124, 0.010844826, 
      0.20727475, 0.7107859, -0.19160423, 0.2678186, 0.060954493, 0.0471791, -0.09462229, -0.101439096, -0.51381135, 0.18143599, -0.4514008, -0.3855419, -0.07337942, -0.071828105, 0.32775605, -0.39448452, 0.30547214, 0.41074327, -0.0665693, 0.24603371, -0.56691974, 0.31382725, -0.50026065, 0.17172787, -0.23183842, -0.28332424, -0.43399972, 0.17262593, 0.14488856, -0.23856215, 0.090194754, -0.17582184, 0.1799406, 0.2926947, -0.0065751052, 0.34280935, 0.0729649, 0.10405836, -0.5847167, -0.26127157, 0.17947093, 0.3617622, 0.16026905, -0.12749536, -0.08876512, 0.17031094, -0.27337608, -0.013942749, 
      0.21798061, -0.024278305, -0.45448065, -0.7835668, -0.0014776988, -0.42699918, -0.6226331, 0.06859, -0.4073471, -0.013574338, -0.27035454, -0.1235234, -0.19551161, 0.43392676, -0.19786318, 0.18843408, 0.09325936, -0.19536705, -0.018174151, -0.39158064, 0.1438622, 0.51706326, 0.07165538, 0.31526235, -0.2017918, -0.6001302, -0.8273044, 0.057214566, -0.3357815, -0.60877496, -0.007436287, -0.059009705, 0.20760801, -0.9196432, 0.08127579, -0.08668095, -0.07939293, 0.22910306, 0.87123734, -0.40429175, -0.17506203, 0.071745165, 0.03610396, 0.6749094, -0.0010482145, 0.29762214, -0.073687226, 0.28717422, 
      0.23985596, 0.22099042, -0.16671588, -0.098301016, 0.22271332, -0.34084603, 0.19370347, 0.20960493, -0.34816524, 0.06889991, 0.046742216, 0.06280888, -0.28508258, -0.40458828, 0.26739743, 0.2000107, -0.22326541, -0.42703545, -0.13270004, -0.22250429, -0.03371752, 0.026883652, 0.044621512, -0.0007646071, 0.20961255, -0.24871475, 0.09323924, 0.08925097, 0.044484735, -0.635433, -0.054663487, -0.14380991, -0.31155816, 0.106311195, 0.6247801, 0.22185367, -0.07387966, -0.07783609, 0.12302069, 0.1276412, -0.20198913, 0.0024636434, -0.21591869, -0.32856792, -0.093851045, -0.039838813, -0.19717225, -0.29588097, 
      -0.2784415, -0.7467973, -0.37535304, -0.37171012, 0.093731515, 0.7802157, -0.1356811, 0.6120949, -0.35498285, 0.6115193, 0.58770114, 0.25849676, 1.0752585, 0.11418658, -0.3260622, -0.23005128, -0.5942296, -0.08405137, 0.51287335, 0.14117046, 0.33847842, -0.6332492, 1.2900072, -0.44275737, -0.11869702, 0.22096711, 0.4161184, -0.25552046, -0.47516084, 0.88787, 0.5486296, -0.045404017, -0.3565241, -0.24124463, 0.11186998, -0.23903486, 0.031721022, 0.05689738, 0.3827738, 0.5529997, -0.08481947, 0.27467558, 0.6395105, -0.25318944, -0.02559216, 0.12866803, -0.055642683, -0.33875972, 
      0.53794825, 0.04837073, -0.43485618, 0.0904165, -0.1798857, -0.24354906, 0.107308365, -0.064898565, 0.17648019, 0.2032626, -0.30696523, -0.43681452, 0.10605955, 0.0629844, 0.24645461, 0.18410674, -0.3611601, 0.12202614, 0.0525437, 0.34131706, 0.1158214, 0.15153828, 0.31087464, -0.056523994, 0.15952015, 0.40987802, -0.12392176, 0.002516542, -0.114805564, 0.28716192, 0.30326632, 0.3215028, 0.14800625, -0.00089095655, -0.053912014, -0.28722173, 0.434732, 0.06117515, 0.07252018, -0.31040165, -0.1839232, 0.106154636, 0.399518, -0.046306938, 0.06772961, 0.27991465, -0.020527838, 0.20520845, 
      -0.3963925, 0.19006191, 0.021816514, 0.23957013, 0.27716544, -0.22744803, 0.22984482, 0.17000581, -0.45176294, -0.15854326, 0.09395864, 0.0020251896, 0.02407729, 0.12572181, -0.31764433, -0.041897945, -0.1868623, -0.101421714, 0.032907717, -0.17156406, 0.149257, -0.1028442, -0.009806323, 0.29436898, 0.060146306, 0.09459751, 0.13902211, 0.21548319, -0.054140676, -0.1590356, -0.18473725, -0.0062132925, 0.07298218, 0.3262601, -0.041481096, -0.09634405, -0.14896107, -0.15889123, 0.25981444, -0.011060926, -0.25774193, -0.09672011, 0.44437128, 0.3249228, 0.16875853, 0.039550204, -0.29874825, 0.044675197, 
      -0.45500702, 0.10436448, 0.037670694, 0.26382253, -0.53757155, 0.3276622, 0.24539834, 0.21960236, -0.12174121, -0.27561897, 0.11983699, -0.15836422, 0.5275954, 0.21102634, 0.005379052, -0.39203638, -0.15168184, -0.121471874, 0.04564997, 0.08353247, 0.41411492, -0.09937978, 0.81701845, -0.28742534, -0.39045388, -0.2507684, 0.3357518, -0.026328942, -0.30628467, 0.52383894, -0.10799475, 0.6820844, -0.16911021, 0.30911344, 0.2949988, -0.060291454, 0.37445304, -0.29679093, -0.062349264, 0.40143085, 0.5580787, 0.30391297, 0.0120282825, 0.35792902, 0.027912956, -0.17071544, 0.1508636, 0.03302149, 
      -0.09139762, 0.1266033, -0.086748466, -0.64231426, 0.042533386, -0.09425409, -0.28185946, 0.17736901, -0.33383638, -0.3216961, 0.071212225, -0.12670992, -0.18995848, 0.40292397, -0.05898073, 0.003989044, 0.041317783, -0.07742622, -0.100309394, -0.17230904, 0.19614112, 0.2929404, -0.033950467, 0.15351193, -0.16737942, -0.6894264, -0.9695455, 0.27005982, -0.01809479, -0.29990935, -0.17844959, -0.10828515, 0.06749042, -0.5365261, 0.45313224, -0.19297186, -0.031094769, 0.494707, 0.5595586, -0.36976168, -0.020598156, -0.15849431, 0.02613671, 0.14074874, -0.12142973, 0.42253867, -0.49428204, 0.20786375; 
    b1 << -0.27163863, 
      0.032091003, 
      0.1012237, 
      -0.034922123, 
      -0.18592212, 
      0.06281815, 
      0.07636818, 
      -0.13781403, 
      0.29423276, 
      0.09453387, 
      0.31851953, 
      0.33794674, 
      -0.08681835, 
      -0.56054866, 
      0.1220036, 
      -0.0927638, 
      -0.003194892, 
      0.052491397; 

    w2 << -0.35073787, -0.09447506, 0.23767403, -0.22414659, -0.29175106, -0.26600692, 0.1718134, 0.22050992, 0.24003847, -0.24112575, 0.3195639, 0.2881509, -0.38434434, 0.32703742, 0.25549, -0.3122288, -0.27996257, -0.2835737; 
    b2 << -0.2992067; 

    Layer<ADScalar> layer0 = { w0, b0 };
    Layer<ADScalar> layer1 = { w1, b1 };
    Layer<ADScalar> layer2 = { w2, b2 };

    InferenceNeuralNetwork<ADScalar> test_model;
    test_model.layers.push_back(layer0);
    test_model.layers.push_back(layer1);
    test_model.layers.push_back(layer2);

    ADFun generatedModel = tapeADNeuralNetInference(test_model, 3);
    generateCompileCLib("codegen_test_model", generatedModel);

    /***************************************************************************
    *                      Generated code evaluation
    ***************************************************************************/
    // Load the model from the compiled library
    const std::string LIBRARY_NAME = "./libCGcodegen_test_model";
    const std::string LIBRARY_NAME_EXT = LIBRARY_NAME + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION;
    CppAD::cg::LinuxDynamicLib<double> dynamicLib(LIBRARY_NAME_EXT);
    std::unique_ptr<CppAD::cg::GenericModel<double> > model = dynamicLib.model("codegen_test_model");

    // Input : Get a random config.
    Eigen::Matrix<double, Eigen::Dynamic, 1> X_test;
    X_test = 3.1415*Eigen::Matrix<double,6,1>::Random(6,1);
    
    // Output : distance
    Eigen::Matrix<double, Eigen::Dynamic, 1> Y_test;
    Y_test.resize(4);
    
    // Function evaluation with start and stop timestamps
    auto start_cg = high_resolution_clock::now();
    for (int k = 0; k<1e6; k++)
    {
        X_test = 3.1415*Eigen::Matrix<double,6,1>::Random(6,1);
        model->ForwardZero(X_test, Y_test);
    }
    auto stop_cg = high_resolution_clock::now(); 
    auto duration_cg = duration_cast<microseconds>(stop_cg - start_cg); 

    std::cout << "\nTime taken by function (1e6 executions): " << ((int)duration_cg.count()) << " microseconds" << std::endl; 

}