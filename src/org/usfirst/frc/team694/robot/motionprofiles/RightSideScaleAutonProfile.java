package org.usfirst.frc.team694.robot.motionprofiles;

public interface RightSideScaleAutonProfile {
	double[][] leftPoints = 
		{		//{Position, Velocity(RPM), Duration}
				{0.000028, 0.211956, 10},
				{0.000066, 0.147919, 10},
				{0.000316, 0.953804, 10},
				{0.000760, 1.695651, 10},
				{0.001454, 2.649455, 10},
				{0.002453, 3.815215, 10},
				{0.003812, 5.192932, 10},
				{0.005588, 6.782605, 10},
				{0.007835, 8.584234, 10},
				{0.010610, 10.597820, 10},
				{0.013967, 12.823362, 10},
				{0.017934, 15.154883, 10},
				{0.022512, 17.486403, 10},
				{0.027701, 19.817924, 10},
				{0.033499, 22.149444, 10},
				{0.039908, 24.480965, 10},
				{0.046928, 26.812485, 10},
				{0.054558, 29.144006, 10},
				{0.062798, 31.475526, 10},
				{0.071649, 33.807046, 10},
				{0.081110, 36.138567, 10},
				{0.091181, 38.470087, 10},
				{0.101863, 40.801608, 10},
				{0.113155, 43.133128, 10},
				{0.125058, 45.464649, 10},
				{0.137571, 47.796169, 10},
				{0.150694, 50.127690, 10},
				{0.164428, 52.459210, 10},
				{0.178772, 54.790730, 10},
				{0.193727, 57.122251, 10},
				{0.209292, 59.453771, 10},
				{0.225467, 61.785292, 10},
				{0.242253, 64.116812, 10},
				{0.259649, 66.448333, 10},
				{0.277655, 68.779853, 10},
				{0.296272, 71.111374, 10},
				{0.315500, 73.442894, 10},
				{0.335337, 75.774414, 10},
				{0.355785, 78.105935, 10},
				{0.376844, 80.437455, 10},
				{0.398513, 82.768976, 10},
				{0.420792, 85.100496, 10},
				{0.443682, 87.432017, 10},
				{0.467182, 89.763537, 10},
				{0.491292, 92.095058, 10},
				{0.516013, 94.426578, 10},
				{0.541344, 96.758098, 10},
				{0.567286, 99.089619, 10},
				{0.593838, 101.421139, 10},
				{0.621000, 103.752660, 10},
				{0.648773, 106.084180, 10},
				{0.677156, 108.415701, 10},
				{0.706150, 110.747221, 10},
				{0.735754, 113.078742, 10},
				{0.765968, 115.410262, 10},
				{0.796793, 117.741782, 10},
				{0.828228, 120.073303, 10},
				{0.860273, 122.404823, 10},
				{0.892929, 124.736344, 10},
				{0.926196, 127.067864, 10},
				{0.960072, 129.399385, 10},
				{0.994559, 131.730905, 10},
				{1.029657, 134.062426, 10},
				{1.065365, 136.393946, 10},
				{1.101683, 138.725466, 10},
				{1.138611, 141.056987, 10},
				{1.176150, 143.388507, 10},
				{1.214300, 145.720028, 10},
				{1.253060, 148.051548, 10},
				{1.292430, 150.383069, 10},
				{1.332410, 152.714589, 10},
				{1.373001, 155.046110, 10},
				{1.414203, 157.377630, 10},
				{1.456015, 159.709150, 10},
				{1.498437, 162.040671, 10},
				{1.541469, 164.372191, 10},
				{1.585112, 166.703712, 10},
				{1.629366, 169.035232, 10},
				{1.674229, 171.366753, 10},
				{1.719703, 173.698273, 10},
				{1.765788, 176.029794, 10},
				{1.812483, 178.361314, 10},
				{1.859788, 180.692834, 10},
				{1.907704, 183.024355, 10},
				{1.956230, 185.355875, 10},
				{2.005366, 187.687396, 10},
				{2.055113, 190.018916, 10},
				{2.105470, 192.350437, 10},
				{2.156438, 194.681957, 10},
				{2.208016, 197.013478, 10},
				{2.260204, 199.344998, 10},
				{2.313003, 201.676518, 10},
				{2.366412, 204.008039, 10},
				{2.420432, 206.339559, 10},
				{2.475062, 208.671080, 10},
				{2.530302, 211.002600, 10},
				{2.586153, 213.334121, 10},
				{2.642614, 215.665641, 10},
				{2.699685, 217.997162, 10},
				{2.757367, 220.328682, 10},
				{2.815660, 222.660202, 10},
				{2.874562, 224.991723, 10},
				{2.934075, 227.323243, 10},
				{2.994199, 229.654764, 10},
				{3.054933, 231.986284, 10},
				{3.116277, 234.317805, 10},
				{3.178232, 236.649325, 10},
				{3.240797, 238.980846, 10},
				{3.303972, 241.312366, 10},
				{3.367758, 243.643886, 10},
				{3.432154, 245.975407, 10},
				{3.497161, 248.306927, 10},
				{3.562778, 250.638448, 10},
				{3.629005, 252.969968, 10},
				{3.695843, 255.301489, 10},
				{3.763291, 257.633009, 10},
				{3.831350, 259.964530, 10},
				{3.900019, 262.296050, 10},
				{3.969298, 264.627570, 10},
				{4.039188, 266.959091, 10},
				{4.109688, 269.290611, 10},
				{4.180798, 271.622132, 10},
				{4.252519, 273.953652, 10},
				{4.324851, 276.285173, 10},
				{4.397792, 278.616693, 10},
				{4.471344, 280.948214, 10},
				{4.545507, 283.279734, 10},
				{4.620280, 285.611254, 10},
				{4.695635, 287.836797, 10},
				{4.771518, 289.850383, 10},
				{4.847872, 291.652012, 10},
				{4.924643, 293.241685, 10},
				{5.001774, 294.619402, 10},
				{5.079210, 295.785162, 10},
				{5.156896, 296.738966, 10},
				{5.234776, 297.480813, 10},
				{5.312795, 298.010704, 10},
				{5.390898, 298.328639, 10},
				{5.469028, 298.434617, 10},
				{5.547158, 298.434617, 10},
				{5.625288, 298.434617, 10},
				{5.703418, 298.434617, 10},
				{5.781548, 298.434617, 10},
				{5.859678, 298.434617, 10},
				{5.937808, 298.434617, 10},
				{6.015938, 298.434617, 10},
				{6.094068, 298.434617, 10},
				{6.172198, 298.434617, 10},
				{6.250328, 298.434617, 10},
				{6.328458, 298.434617, 10},
				{6.406588, 298.434617, 10},
				{6.484718, 298.434617, 10},
				{6.562848, 298.434617, 10},
				{6.640978, 298.434617, 10},
				{6.719108, 298.434617, 10},
				{6.797238, 298.434617, 10},
				{6.875368, 298.434617, 10},
				{6.953498, 298.434617, 10},
				{7.031628, 298.434617, 10},
				{7.109758, 298.434617, 10},
				{7.187888, 298.434617, 10},
				{7.266018, 298.434617, 10},
				{7.344148, 298.434617, 10},
				{7.422278, 298.434617, 10},
				{7.500408, 298.434617, 10},
				{7.578538, 298.434617, 10},
				{7.656668, 298.434617, 10},
				{7.734798, 298.434617, 10},
				{7.812928, 298.434617, 10},
				{7.891058, 298.434617, 10},
				{7.969188, 298.434617, 10},
				{8.047318, 298.434617, 10},
				{8.125448, 298.434617, 10},
				{8.203578, 298.434617, 10},
				{8.281708, 298.434617, 10},
				{8.359838, 298.434617, 10},
				{8.437968, 298.434617, 10},
				{8.516098, 298.434617, 10},
				{8.594228, 298.434617, 10},
				{8.672358, 298.434617, 10},
				{8.750488, 298.434617, 10},
				{8.828618, 298.434617, 10},
				{8.906748, 298.434617, 10},
				{8.984878, 298.434617, 10},
				{9.063008, 298.434617, 10},
				{9.141138, 298.434617, 10},
				{9.219268, 298.434617, 10},
				{9.297398, 298.434617, 10},
				{9.375528, 298.434617, 10},
				{9.453658, 298.434617, 10},
				{9.531788, 298.434617, 10},
				{9.609918, 298.434617, 10},
				{9.688048, 298.434617, 10},
				{9.766178, 298.434617, 10},
				{9.844308, 298.434617, 10},
				{9.922438, 298.434617, 10},
				{10.000428, 297.899856, 10},
				{10.078558, 298.434617, 10},
				{10.156688, 298.434617, 10},
				{10.234818, 298.434617, 10},
				{10.312948, 298.434617, 10},
				{10.391078, 298.434617, 10},
				{10.469208, 298.434617, 10},
				{10.547338, 298.434617, 10},
				{10.625468, 298.434617, 10},
				{10.703598, 298.434617, 10},
				{10.781728, 298.434617, 10},
				{10.859858, 298.434617, 10},
				{10.937988, 298.434617, 10},
				{11.016118, 298.434617, 10},
				{11.094248, 298.434617, 10},
				{11.172378, 298.434617, 10},
				{11.250508, 298.434617, 10},
				{11.328638, 298.434617, 10},
				{11.406768, 298.434617, 10},
				{11.484898, 298.434617, 10},
				{11.563028, 298.434617, 10},
				{11.641158, 298.434617, 10},
				{11.719288, 298.434617, 10},
				{11.797418, 298.434617, 10},
				{11.875548, 298.434617, 10},
				{11.953678, 298.434617, 10},
				{12.031808, 298.434617, 10},
				{12.109938, 298.434617, 10},
				{12.188068, 298.434617, 10},
				{12.266198, 298.434617, 10},
				{12.344328, 298.434617, 10},
				{12.422458, 298.434617, 10},
				{12.500588, 298.434617, 10},
				{12.578718, 298.434617, 10},
				{12.656848, 298.434617, 10},
				{12.734978, 298.434617, 10},
				{12.813108, 298.434617, 10},
				{12.891238, 298.434617, 10},
				{12.969368, 298.434617, 10},
				{13.047498, 298.434617, 10},
				{13.125628, 298.434617, 10},
				{13.203758, 298.434617, 10},
				{13.281888, 298.434617, 10},
				{13.360018, 298.434617, 10},
				{13.438148, 298.434617, 10},
				{13.516278, 298.434617, 10},
				{13.594408, 298.434617, 10},
				{13.672538, 298.434617, 10},
				{13.750668, 298.434617, 10},
				{13.828798, 298.434617, 10},
				{13.906928, 298.434617, 10},
				{13.985058, 298.434617, 10},
				{14.063188, 298.434617, 10},
				{14.141318, 298.434617, 10},
				{14.219448, 298.434617, 10},
				{14.297578, 298.434617, 10},
				{14.375708, 298.434617, 10},
				{14.453838, 298.434617, 10},
				{14.531968, 298.434617, 10},
				{14.610098, 298.434617, 10},
				{14.688228, 298.434617, 10},
				{14.766358, 298.434617, 10},
				{14.844488, 298.434617, 10},
				{14.922618, 298.434617, 10},
				{15.000748, 298.434617, 10},
				{15.078878, 298.434617, 10},
				{15.157008, 298.434617, 10},
				{15.235138, 298.434617, 10},
				{15.313268, 298.434617, 10},
				{15.391398, 298.434617, 10},
				{15.469528, 298.434617, 10},
				{15.547658, 298.434617, 10},
				{15.625788, 298.434617, 10},
				{15.703918, 298.434617, 10},
				{15.782048, 298.434617, 10},
				{15.860178, 298.434617, 10},
				{15.938308, 298.434617, 10},
				{16.016438, 298.434617, 10},
				{16.094568, 298.434617, 10},
				{16.172698, 298.434617, 10},
				{16.250828, 298.434617, 10},
				{16.328958, 298.434617, 10},
				{16.407088, 298.434617, 10},
				{16.485218, 298.434617, 10},
				{16.563348, 298.434617, 10},
				{16.641478, 298.434617, 10},
				{16.719608, 298.434617, 10},
				{16.797738, 298.434617, 10},
				{16.875868, 298.434617, 10},
				{16.953998, 298.434617, 10},
				{17.032128, 298.434617, 10},
				{17.110258, 298.434617, 10},
				{17.188388, 298.434617, 10},
				{17.266518, 298.434617, 10},
				{17.344648, 298.434617, 10},
				{17.422778, 298.434617, 10},
				{17.500908, 298.434617, 10},
				{17.579038, 298.434617, 10},
				{17.657168, 298.434617, 10},
				{17.735298, 298.434617, 10},
				{17.813428, 298.434617, 10},
				{17.891558, 298.434617, 10},
				{17.969688, 298.434617, 10},
				{18.047818, 298.434617, 10},
				{18.125948, 298.434617, 10},
				{18.204078, 298.434617, 10},
				{18.282208, 298.434617, 10},
				{18.360338, 298.434617, 10},
				{18.438468, 298.434617, 10},
				{18.516598, 298.434617, 10},
				{18.594728, 298.434617, 10},
				{18.672858, 298.434617, 10},
				{18.750988, 298.434617, 10},
				{18.829118, 298.434617, 10},
				{18.907248, 298.434617, 10},
				{18.985378, 298.434617, 10},
				{19.063508, 298.434617, 10},
				{19.141638, 298.434617, 10},
				{19.219768, 298.434617, 10},
				{19.297898, 298.434617, 10},
				{19.376028, 298.434617, 10},
				{19.454158, 298.434617, 10},
				{19.532288, 298.434617, 10},
				{19.610418, 298.434617, 10},
				{19.688548, 298.434617, 10},
				{19.766678, 298.434617, 10},
				{19.844808, 298.434617, 10},
				{19.922938, 298.434617, 10},
				{20.001068, 298.434617, 10},
				{20.079198, 298.434617, 10},
				{20.157328, 298.434617, 10},
				{20.235458, 298.434617, 10},
				{20.313588, 298.434617, 10},
				{20.391718, 298.434617, 10},
				{20.469848, 298.434617, 10},
				{20.547978, 298.434617, 10},
				{20.626108, 298.434617, 10},
				{20.704238, 298.434617, 10},
				{20.782368, 298.434617, 10},
				{20.860498, 298.434617, 10},
				{20.938628, 298.434617, 10},
				{21.016758, 298.434617, 10},
				{21.094888, 298.434617, 10},
				{21.173018, 298.434617, 10},
				{21.251148, 298.434617, 10},
				{21.329278, 298.434617, 10},
				{21.407408, 298.434617, 10},
				{21.485538, 298.434617, 10},
				{21.563668, 298.434617, 10},
				{21.641798, 298.434617, 10},
				{21.719928, 298.434617, 10},
				{21.798058, 298.434617, 10},
				{21.876188, 298.434617, 10},
				{21.954318, 298.434617, 10},
				{22.032448, 298.434617, 10},
				{22.110578, 298.434617, 10},
				{22.188708, 298.434617, 10},
				{22.266838, 298.434617, 10},
				{22.344968, 298.434617, 10},
				{22.423098, 298.434617, 10},
				{22.501228, 298.434617, 10},
				{22.579358, 298.434617, 10},
				{22.657488, 298.434617, 10},
				{22.735618, 298.434617, 10},
				{22.813748, 298.434617, 10},
				{22.891878, 298.434617, 10},
				{22.970008, 298.434617, 10},
				{23.048138, 298.434617, 10},
				{23.126268, 298.434617, 10},
				{23.204398, 298.434617, 10},
				{23.282528, 298.434617, 10},
				{23.360631, 298.333752, 10},
				{23.438654, 298.026045, 10},
				{23.516541, 297.506381, 10},
				{23.594237, 296.774761, 10},
				{23.671685, 295.831185, 10},
				{23.748831, 294.675652, 10},
				{23.825619, 293.308162, 10},
				{23.901993, 291.728717, 10},
				{23.977899, 289.937314, 10},
				{24.045591, 258.565610, 10},
				{24.109284, 243.289681, 10},
				{24.172005, 239.575921, 10},
				{24.233739, 235.807095, 10},
				{24.294473, 231.986579, 10},
				{24.354193, 228.113577, 10},
				{24.412885, 224.187538, 10},
				{24.470536, 220.208210, 10},
				{24.527130, 216.175696, 10},
				{24.582656, 212.090526, 10},
				{24.637098, 207.953725, 10},
				{24.690444, 203.766894, 10},
				{24.742681, 199.532299, 10},
				{24.793798, 195.252958, 10},
				{24.843784, 190.932741, 10},
				{24.892630, 186.576459, 10},
				{24.940327, 182.189964, 10},
				{24.986870, 177.780240, 10},
				{25.032254, 173.355475, 10},
				{25.076479, 168.925131, 10},
				{25.119545, 164.499986, 10},
				{25.161457, 160.092144, 10},
				{25.202223, 155.715015, 10},
				{25.241855, 151.383248, 10},
				{25.280369, 147.112614, 10},
				{25.317785, 142.919834, 10},
				{25.354129, 138.822346, 10},
				{25.389429, 134.838015, 10},
				{25.423721, 130.984784, 10},
				{25.457043, 127.280280, 10},
				{25.489438, 123.741372, 10},
				{25.520955, 120.383725, 10},
				{25.551643, 117.221334, 10},
				{25.581558, 114.266096, 10},
				{25.610756, 111.527415, 10},
				{25.639295, 109.011885, 10},
				{25.667235, 106.723052, 10},
				{25.694635, 104.661290, 10},
				{25.721555, 102.823776, 10},
				{25.748050, 101.204591, 10},
				{25.774176, 99.794919, 10},
				{25.799985, 98.583347, 10},
				{25.825525, 97.556231, 10},
				{25.850841, 96.698130, 10},
				{25.875972, 95.992252, 10},
				{25.900953, 95.420926, 10},
				{25.925815, 94.966040, 10},
				{25.950584, 94.609461, 10},
				{25.975280, 94.333399, 10},
				{25.999921, 94.120724, 10},
				{26.024518, 93.955213, 10},
				{26.049081, 93.821749, 10},
				{26.073613, 93.706449, 10},
				{26.098116, 93.596750, 10},
				{26.122590, 93.481442, 10},
				{26.147029, 93.350665, 10},
				{26.171428, 93.195871, 10},
				{26.195778, 93.009766, 10},
				{26.220069, 92.786234, 10},
				{26.244291, 92.520240, 10},
				{26.268431, 92.207746, 10},
				{26.292476, 91.845600, 10},
				{26.316412, 91.431447, 10},
				{26.340227, 90.963630, 10},
				{26.363904, 90.441102, 10},
				{26.387430, 89.863342, 10},
				{26.410791, 89.230276, 10},
				{26.433971, 88.542209, 10},
				{26.456957, 87.799763, 10},
				{26.479734, 87.003819, 10},
				{26.502290, 86.155469, 10},
				{26.524610, 85.255971, 10},
				{26.546681, 84.306716, 10},
				{26.568492, 83.309192, 10},
				{26.590028, 82.264955, 10},
				{26.611280, 81.175613, 10},
				{26.632235, 80.042797, 10},
				{26.652883, 78.868153, 10},
				{26.673213, 77.653322, 10},
				{26.693214, 76.399933, 10},
				{26.712878, 75.109591, 10},
				{26.732194, 73.783873, 10},
				{26.751155, 72.424321, 10},
				{26.769751, 71.032437, 10},
				{26.787975, 69.609681, 10},
				{26.805818, 68.157468, 10},
				{26.823275, 66.677167, 10},
				{26.840336, 65.170100, 10},
				{26.856996, 63.637543, 10},
				{26.873249, 62.080724, 10},
				{26.889088, 60.500826, 10},
				{26.904508, 58.898986, 10},
				{26.919503, 57.276298, 10},
				{26.934068, 55.633811, 10},
				{26.948198, 53.972535, 10},
				{26.961888, 52.293439, 10},
				{26.975134, 50.597453, 10},
				{26.987933, 48.885471, 10},
				{27.000279, 47.158352, 10},
				{27.012169, 45.416921, 10},
				{27.023599, 43.661973, 10},
				{27.034567, 41.894271, 10},
				{27.045069, 40.114549, 10},
				{27.055102, 38.323516, 10},
				{27.064664, 36.521855, 10},
				{27.073751, 34.710225, 10},
				{27.082361, 32.889262, 10},
				{27.090493, 31.059582, 10},
				{27.098143, 29.221780, 10},
				{27.105310, 27.376436, 10},
				{27.111992, 25.524109, 10},
				{27.118188, 23.665345, 10},
				{27.123895, 21.800675, 10},
				{27.129113, 19.930617, 10},
				{27.133840, 18.055677, 10},
				{27.138075, 16.176349, 10},
				{27.141817, 14.293120, 10},
				{27.145065, 12.406465, 10},
				{27.147818, 10.516853, 10},
				{27.150098, 8.706798, 10},
				{27.151947, 7.063084, 10},
				{27.153410, 5.590436, 10},
				{27.154533, 4.289308, 10},
				{27.155360, 3.160052, 10},
				{27.155937, 2.202933, 10},
				{27.156308, 1.418143, 10},
				{27.156519, 0.805809, 10},
				{27.156615, 0.366012, 10},
				{27.156641, 0.098791, 10},
				{27.156642, 0.004164, 10},
				{27.156642, 0.000000, 10},
		};
	double[][] rightPoints = 
		{		//{Position, Velocity(RPM), Duration}
				{0.000028, 0.211956, 10},
				{0.000066, 0.147919, 10},
				{0.000316, 0.953804, 10},
				{0.000760, 1.695651, 10},
				{0.001454, 2.649455, 10},
				{0.002453, 3.815215, 10},
				{0.003812, 5.192932, 10},
				{0.005588, 6.782605, 10},
				{0.007835, 8.584234, 10},
				{0.010610, 10.597820, 10},
				{0.013967, 12.823362, 10},
				{0.017934, 15.154883, 10},
				{0.022512, 17.486403, 10},
				{0.027701, 19.817924, 10},
				{0.033499, 22.149444, 10},
				{0.039908, 24.480965, 10},
				{0.046928, 26.812485, 10},
				{0.054558, 29.144006, 10},
				{0.062798, 31.475526, 10},
				{0.071649, 33.807046, 10},
				{0.081110, 36.138567, 10},
				{0.091181, 38.470087, 10},
				{0.101863, 40.801608, 10},
				{0.113155, 43.133128, 10},
				{0.125058, 45.464649, 10},
				{0.137571, 47.796169, 10},
				{0.150694, 50.127690, 10},
				{0.164428, 52.459210, 10},
				{0.178772, 54.790730, 10},
				{0.193727, 57.122251, 10},
				{0.209292, 59.453771, 10},
				{0.225467, 61.785292, 10},
				{0.242253, 64.116812, 10},
				{0.259649, 66.448333, 10},
				{0.277655, 68.779853, 10},
				{0.296272, 71.111374, 10},
				{0.315500, 73.442894, 10},
				{0.335337, 75.774414, 10},
				{0.355785, 78.105935, 10},
				{0.376844, 80.437455, 10},
				{0.398513, 82.768976, 10},
				{0.420792, 85.100496, 10},
				{0.443682, 87.432017, 10},
				{0.467182, 89.763537, 10},
				{0.491292, 92.095058, 10},
				{0.516013, 94.426578, 10},
				{0.541344, 96.758098, 10},
				{0.567286, 99.089619, 10},
				{0.593838, 101.421139, 10},
				{0.621000, 103.752660, 10},
				{0.648773, 106.084180, 10},
				{0.677156, 108.415701, 10},
				{0.706150, 110.747221, 10},
				{0.735754, 113.078742, 10},
				{0.765968, 115.410262, 10},
				{0.796793, 117.741782, 10},
				{0.828228, 120.073303, 10},
				{0.860273, 122.404823, 10},
				{0.892929, 124.736344, 10},
				{0.926196, 127.067864, 10},
				{0.960072, 129.399385, 10},
				{0.994559, 131.730905, 10},
				{1.029657, 134.062426, 10},
				{1.065365, 136.393946, 10},
				{1.101683, 138.725466, 10},
				{1.138611, 141.056987, 10},
				{1.176150, 143.388507, 10},
				{1.214300, 145.720028, 10},
				{1.253060, 148.051548, 10},
				{1.292430, 150.383069, 10},
				{1.332410, 152.714589, 10},
				{1.373001, 155.046110, 10},
				{1.414203, 157.377630, 10},
				{1.456015, 159.709150, 10},
				{1.498437, 162.040671, 10},
				{1.541469, 164.372191, 10},
				{1.585112, 166.703712, 10},
				{1.629366, 169.035232, 10},
				{1.674229, 171.366753, 10},
				{1.719703, 173.698273, 10},
				{1.765788, 176.029794, 10},
				{1.812483, 178.361314, 10},
				{1.859788, 180.692834, 10},
				{1.907704, 183.024355, 10},
				{1.956230, 185.355875, 10},
				{2.005366, 187.687396, 10},
				{2.055113, 190.018916, 10},
				{2.105470, 192.350437, 10},
				{2.156438, 194.681957, 10},
				{2.208016, 197.013478, 10},
				{2.260204, 199.344998, 10},
				{2.313003, 201.676518, 10},
				{2.366412, 204.008039, 10},
				{2.420432, 206.339559, 10},
				{2.475062, 208.671080, 10},
				{2.530302, 211.002600, 10},
				{2.586153, 213.334121, 10},
				{2.642614, 215.665641, 10},
				{2.699685, 217.997162, 10},
				{2.757367, 220.328682, 10},
				{2.815660, 222.660202, 10},
				{2.874562, 224.991723, 10},
				{2.934075, 227.323243, 10},
				{2.994199, 229.654764, 10},
				{3.054933, 231.986284, 10},
				{3.116277, 234.317805, 10},
				{3.178232, 236.649325, 10},
				{3.240797, 238.980846, 10},
				{3.303972, 241.312366, 10},
				{3.367758, 243.643886, 10},
				{3.432154, 245.975407, 10},
				{3.497161, 248.306927, 10},
				{3.562778, 250.638448, 10},
				{3.629005, 252.969968, 10},
				{3.695843, 255.301489, 10},
				{3.763291, 257.633009, 10},
				{3.831350, 259.964530, 10},
				{3.900019, 262.296050, 10},
				{3.969298, 264.627570, 10},
				{4.039188, 266.959091, 10},
				{4.109688, 269.290611, 10},
				{4.180798, 271.622132, 10},
				{4.252519, 273.953652, 10},
				{4.324851, 276.285173, 10},
				{4.397792, 278.616693, 10},
				{4.471344, 280.948214, 10},
				{4.545507, 283.279734, 10},
				{4.620280, 285.611254, 10},
				{4.695635, 287.836797, 10},
				{4.771518, 289.850383, 10},
				{4.847872, 291.652012, 10},
				{4.924643, 293.241685, 10},
				{5.001774, 294.619402, 10},
				{5.079210, 295.785162, 10},
				{5.156896, 296.738966, 10},
				{5.234776, 297.480813, 10},
				{5.312795, 298.010704, 10},
				{5.390898, 298.328639, 10},
				{5.469028, 298.434617, 10},
				{5.547158, 298.434617, 10},
				{5.625288, 298.434617, 10},
				{5.703418, 298.434617, 10},
				{5.781548, 298.434617, 10},
				{5.859678, 298.434617, 10},
				{5.937808, 298.434617, 10},
				{6.015938, 298.434617, 10},
				{6.094068, 298.434617, 10},
				{6.172198, 298.434617, 10},
				{6.250328, 298.434617, 10},
				{6.328458, 298.434617, 10},
				{6.406588, 298.434617, 10},
				{6.484718, 298.434617, 10},
				{6.562848, 298.434617, 10},
				{6.640978, 298.434617, 10},
				{6.719108, 298.434617, 10},
				{6.797238, 298.434617, 10},
				{6.875368, 298.434617, 10},
				{6.953498, 298.434617, 10},
				{7.031628, 298.434617, 10},
				{7.109758, 298.434617, 10},
				{7.187888, 298.434617, 10},
				{7.266018, 298.434617, 10},
				{7.344148, 298.434617, 10},
				{7.422278, 298.434617, 10},
				{7.500408, 298.434617, 10},
				{7.578538, 298.434617, 10},
				{7.656668, 298.434617, 10},
				{7.734798, 298.434617, 10},
				{7.812928, 298.434617, 10},
				{7.891058, 298.434617, 10},
				{7.969188, 298.434617, 10},
				{8.047318, 298.434617, 10},
				{8.125448, 298.434617, 10},
				{8.203578, 298.434617, 10},
				{8.281708, 298.434617, 10},
				{8.359838, 298.434617, 10},
				{8.437968, 298.434617, 10},
				{8.516098, 298.434617, 10},
				{8.594228, 298.434617, 10},
				{8.672358, 298.434617, 10},
				{8.750488, 298.434617, 10},
				{8.828618, 298.434617, 10},
				{8.906748, 298.434617, 10},
				{8.984878, 298.434617, 10},
				{9.063008, 298.434617, 10},
				{9.141138, 298.434617, 10},
				{9.219268, 298.434617, 10},
				{9.297398, 298.434617, 10},
				{9.375528, 298.434617, 10},
				{9.453658, 298.434617, 10},
				{9.531788, 298.434617, 10},
				{9.609918, 298.434617, 10},
				{9.688048, 298.434617, 10},
				{9.766178, 298.434617, 10},
				{9.844308, 298.434617, 10},
				{9.922438, 298.434617, 10},
				{10.000428, 297.899856, 10},
				{10.078558, 298.434617, 10},
				{10.156688, 298.434617, 10},
				{10.234818, 298.434617, 10},
				{10.312948, 298.434617, 10},
				{10.391078, 298.434617, 10},
				{10.469208, 298.434617, 10},
				{10.547338, 298.434617, 10},
				{10.625468, 298.434617, 10},
				{10.703598, 298.434617, 10},
				{10.781728, 298.434617, 10},
				{10.859858, 298.434617, 10},
				{10.937988, 298.434617, 10},
				{11.016118, 298.434617, 10},
				{11.094248, 298.434617, 10},
				{11.172378, 298.434617, 10},
				{11.250508, 298.434617, 10},
				{11.328638, 298.434617, 10},
				{11.406768, 298.434617, 10},
				{11.484898, 298.434617, 10},
				{11.563028, 298.434617, 10},
				{11.641158, 298.434617, 10},
				{11.719288, 298.434617, 10},
				{11.797418, 298.434617, 10},
				{11.875548, 298.434617, 10},
				{11.953678, 298.434617, 10},
				{12.031808, 298.434617, 10},
				{12.109938, 298.434617, 10},
				{12.188068, 298.434617, 10},
				{12.266198, 298.434617, 10},
				{12.344328, 298.434617, 10},
				{12.422458, 298.434617, 10},
				{12.500588, 298.434617, 10},
				{12.578718, 298.434617, 10},
				{12.656848, 298.434617, 10},
				{12.734978, 298.434617, 10},
				{12.813108, 298.434617, 10},
				{12.891238, 298.434617, 10},
				{12.969368, 298.434617, 10},
				{13.047498, 298.434617, 10},
				{13.125628, 298.434617, 10},
				{13.203758, 298.434617, 10},
				{13.281888, 298.434617, 10},
				{13.360018, 298.434617, 10},
				{13.438148, 298.434617, 10},
				{13.516278, 298.434617, 10},
				{13.594408, 298.434617, 10},
				{13.672538, 298.434617, 10},
				{13.750668, 298.434617, 10},
				{13.828798, 298.434617, 10},
				{13.906928, 298.434617, 10},
				{13.985058, 298.434617, 10},
				{14.063188, 298.434617, 10},
				{14.141318, 298.434617, 10},
				{14.219448, 298.434617, 10},
				{14.297578, 298.434617, 10},
				{14.375708, 298.434617, 10},
				{14.453838, 298.434617, 10},
				{14.531968, 298.434617, 10},
				{14.610098, 298.434617, 10},
				{14.688228, 298.434617, 10},
				{14.766358, 298.434617, 10},
				{14.844488, 298.434617, 10},
				{14.922618, 298.434617, 10},
				{15.000748, 298.434617, 10},
				{15.078878, 298.434617, 10},
				{15.157008, 298.434617, 10},
				{15.235138, 298.434617, 10},
				{15.313268, 298.434617, 10},
				{15.391398, 298.434617, 10},
				{15.469528, 298.434617, 10},
				{15.547658, 298.434617, 10},
				{15.625788, 298.434617, 10},
				{15.703918, 298.434617, 10},
				{15.782048, 298.434617, 10},
				{15.860178, 298.434617, 10},
				{15.938308, 298.434617, 10},
				{16.016438, 298.434617, 10},
				{16.094568, 298.434617, 10},
				{16.172698, 298.434617, 10},
				{16.250828, 298.434617, 10},
				{16.328958, 298.434617, 10},
				{16.407088, 298.434617, 10},
				{16.485218, 298.434617, 10},
				{16.563348, 298.434617, 10},
				{16.641478, 298.434617, 10},
				{16.719608, 298.434617, 10},
				{16.797738, 298.434617, 10},
				{16.875868, 298.434617, 10},
				{16.953998, 298.434617, 10},
				{17.032128, 298.434617, 10},
				{17.110258, 298.434617, 10},
				{17.188388, 298.434617, 10},
				{17.266518, 298.434617, 10},
				{17.344648, 298.434617, 10},
				{17.422778, 298.434617, 10},
				{17.500908, 298.434617, 10},
				{17.579038, 298.434617, 10},
				{17.657168, 298.434617, 10},
				{17.735298, 298.434617, 10},
				{17.813428, 298.434617, 10},
				{17.891558, 298.434617, 10},
				{17.969688, 298.434617, 10},
				{18.047818, 298.434617, 10},
				{18.125948, 298.434617, 10},
				{18.204078, 298.434617, 10},
				{18.282208, 298.434617, 10},
				{18.360338, 298.434617, 10},
				{18.438468, 298.434617, 10},
				{18.516598, 298.434617, 10},
				{18.594728, 298.434617, 10},
				{18.672858, 298.434617, 10},
				{18.750988, 298.434617, 10},
				{18.829118, 298.434617, 10},
				{18.907248, 298.434617, 10},
				{18.985378, 298.434617, 10},
				{19.063508, 298.434617, 10},
				{19.141638, 298.434617, 10},
				{19.219768, 298.434617, 10},
				{19.297898, 298.434617, 10},
				{19.376028, 298.434617, 10},
				{19.454158, 298.434617, 10},
				{19.532288, 298.434617, 10},
				{19.610418, 298.434617, 10},
				{19.688548, 298.434617, 10},
				{19.766678, 298.434617, 10},
				{19.844808, 298.434617, 10},
				{19.922938, 298.434617, 10},
				{20.001068, 298.434617, 10},
				{20.079198, 298.434617, 10},
				{20.157328, 298.434617, 10},
				{20.235458, 298.434617, 10},
				{20.313588, 298.434617, 10},
				{20.391718, 298.434617, 10},
				{20.469848, 298.434617, 10},
				{20.547978, 298.434617, 10},
				{20.626108, 298.434617, 10},
				{20.704238, 298.434617, 10},
				{20.782368, 298.434617, 10},
				{20.860498, 298.434617, 10},
				{20.938628, 298.434617, 10},
				{21.016758, 298.434617, 10},
				{21.094888, 298.434617, 10},
				{21.173018, 298.434617, 10},
				{21.251148, 298.434617, 10},
				{21.329278, 298.434617, 10},
				{21.407408, 298.434617, 10},
				{21.485538, 298.434617, 10},
				{21.563668, 298.434617, 10},
				{21.641798, 298.434617, 10},
				{21.719928, 298.434617, 10},
				{21.798058, 298.434617, 10},
				{21.876188, 298.434617, 10},
				{21.954318, 298.434617, 10},
				{22.032448, 298.434617, 10},
				{22.110578, 298.434617, 10},
				{22.188708, 298.434617, 10},
				{22.266838, 298.434617, 10},
				{22.344968, 298.434617, 10},
				{22.423098, 298.434617, 10},
				{22.501228, 298.434617, 10},
				{22.579358, 298.434617, 10},
				{22.657488, 298.434617, 10},
				{22.735618, 298.434617, 10},
				{22.813748, 298.434617, 10},
				{22.891878, 298.434617, 10},
				{22.970008, 298.434617, 10},
				{23.048138, 298.434617, 10},
				{23.126268, 298.434617, 10},
				{23.204398, 298.434617, 10},
				{23.282528, 298.434617, 10},
				{23.360631, 298.333752, 10},
				{23.438654, 298.026045, 10},
				{23.516541, 297.506381, 10},
				{23.594237, 296.774761, 10},
				{23.671685, 295.831185, 10},
				{23.748831, 294.675652, 10},
				{23.825619, 293.308162, 10},
				{23.901993, 291.728717, 10},
				{23.977899, 289.937314, 10},
				{24.060857, 316.878574, 10},
				{24.146765, 328.144467, 10},
				{24.232428, 327.205232, 10},
				{24.317856, 326.310824, 10},
				{24.403063, 325.468096, 10},
				{24.488063, 324.677841, 10},
				{24.572871, 323.940611, 10},
				{24.657499, 323.256657, 10},
				{24.741963, 322.625875, 10},
				{24.826274, 322.047735, 10},
				{24.910448, 321.521213, 10},
				{24.994498, 321.044707, 10},
				{25.078435, 320.615950, 10},
				{25.162271, 320.231925, 10},
				{25.246018, 319.888764, 10},
				{25.329684, 319.581656, 10},
				{25.413278, 319.304748, 10},
				{25.496805, 319.051060, 10},
				{25.580270, 318.812405, 10},
				{25.663674, 318.579323, 10},
				{25.747016, 318.341040, 10},
				{25.830290, 318.085453, 10},
				{25.913490, 317.799157, 10},
				{25.996603, 317.467507, 10},
				{26.079613, 317.074737, 10},
				{26.162499, 316.604131, 10},
				{26.245238, 316.038256, 10},
				{26.327799, 315.359252, 10},
				{26.410148, 314.549181, 10},
				{26.492246, 313.590423, 10},
				{26.574049, 312.466112, 10},
				{26.655511, 311.160589, 10},
				{26.736579, 309.659861, 10},
				{26.817201, 307.952035, 10},
				{26.897319, 306.027708, 10},
				{26.976875, 303.880286, 10},
				{27.055809, 301.506223, 10},
				{27.134062, 298.905144, 10},
				{27.211575, 296.079868, 10},
				{27.288292, 293.036309, 10},
				{27.364157, 289.783279, 10},
				{27.439119, 286.332187, 10},
				{27.513129, 282.696669, 10},
				{27.586142, 278.892161, 10},
				{27.658120, 274.935448, 10},
				{27.729027, 270.844195, 10},
				{27.798832, 266.636508, 10},
				{27.867510, 262.330516, 10},
				{27.935040, 257.944002, 10},
				{28.001405, 253.494095, 10},
				{28.066592, 248.997010, 10},
				{28.130593, 244.467866, 10},
				{28.193404, 239.920540, 10},
				{28.255024, 235.367595, 10},
				{28.315452, 230.820239, 10},
				{28.374694, 226.288332, 10},
				{28.432756, 221.780421, 10},
				{28.489646, 217.303800, 10},
				{28.545374, 212.864587, 10},
				{28.599951, 208.467814, 10},
				{28.653389, 204.117523, 10},
				{28.705701, 199.816865, 10},
				{28.756900, 195.568197, 10},
				{28.807002, 191.373175, 10},
				{28.856019, 187.232849, 10},
				{28.903967, 183.147740, 10},
				{28.950860, 179.117923, 10},
				{28.996712, 175.143095, 10},
				{29.041538, 171.222633, 10},
				{29.085352, 167.355658, 10},
				{29.128167, 163.541080, 10},
				{29.169997, 159.777639, 10},
				{29.210854, 156.063947, 10},
				{29.250752, 152.398516, 10},
				{29.289702, 148.779790, 10},
				{29.327717, 145.206163, 10},
				{29.364808, 141.676003, 10},
				{29.400985, 138.187667, 10},
				{29.436260, 134.739511, 10},
				{29.470642, 131.329910, 10},
				{29.504141, 127.957256, 10},
				{29.536767, 124.619974, 10},
				{29.568527, 121.316523, 10},
				{29.599432, 118.045400, 10},
				{29.629487, 114.805147, 10},
				{29.658703, 111.594348, 10},
				{29.687085, 108.411634, 10},
				{29.714641, 105.255684, 10},
				{29.741377, 102.125222, 10},
				{29.767300, 99.019020, 10},
				{29.792416, 95.935894, 10},
				{29.816731, 92.874710, 10},
				{29.840249, 89.834372, 10},
				{29.862977, 86.813831, 10},
				{29.884919, 83.812077, 10},
				{29.906080, 80.828144, 10},
				{29.926464, 77.861099, 10},
				{29.946075, 74.910049, 10},
				{29.964918, 71.974135, 10},
				{29.982996, 69.052532, 10},
				{30.000312, 66.144446, 10},
				{30.016871, 63.249113, 10},
				{30.032675, 60.365799, 10},
				{30.047727, 57.493795, 10},
				{30.062029, 54.632420, 10},
				{30.075586, 51.781013, 10},
				{30.088398, 48.938938, 10},
				{30.100468, 46.105580, 10},
				{30.111799, 43.280343, 10},
				{30.122392, 40.462649, 10},
				{30.132249, 37.651936, 10},
				{30.141372, 34.847660, 10},
				{30.149763, 32.049291, 10},
				{30.157422, 29.256309, 10},
				{30.164352, 26.468210, 10},
				{30.170552, 23.684497, 10},
				{30.176025, 20.904686, 10},
				{30.180771, 18.128301, 10},
				{30.184791, 15.354872, 10},
				{30.188117, 12.703615, 10},
				{30.190813, 10.299703, 10},
				{30.192946, 8.148637, 10},
				{30.194583, 6.249965, 10},
				{30.195788, 4.603333, 10},
				{30.196628, 3.208477, 10},
				{30.197168, 2.065205, 10},
				{30.197476, 1.173389, 10},
				{30.197615, 0.532951, 10},
				{30.197653, 0.143848, 10},
				{30.197654, 0.006063, 10},
				{30.197654, 0.000000, 10},
		};
}
