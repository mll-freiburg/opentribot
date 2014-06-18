#include "Structures/Journal.h"
#include "Fundamental/random.h"
#include <string>
#include <fstream>
#include <sys/types.h>
#include <unistd.h>
#include "ros/ros.h"
#include "Audio/WavePlay.h"
#include "ImageProcessing/Vision.h"
#include "opentribot_messages/VisibleObjectList.h"
#include "opentribot_messages/VisionSignal.h"
#include <termios.h>
#include "ImageProcessing/Formation/ColorTuples.h"
#include "ImageProcessing/Formation/Image.h"



#include <FL/Fl.H>
#include <FL/fl_draw.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_RGB_Image.H>
#include <FL/Fl_Shared_Image.H>
#include <FL/Fl_JPEG_Image.H>
#include <FL/Fl_Box.H>

using namespace std;

//#define DEBUGCMDLINEARG 1

namespace {

void help_text(const char* pname) {
	cerr << "Aufruf: \n" << pname << " [-h|--help] \n" << pname
			<< " [-restart|--restart][-l|-no_rotate_log|--no_rotate_log][Konfigurationsdatei]\n"
			<< "wird keine Konfiribots::gurationsdatei angegeben, wird automatisch\n"
			<< "../config_files/robotcontrol.cfg verwendet\n" << " Optionen: \n"
			<< "-h | --help : diese Zeilen \n"
			<< "-restart | --restart : auto restart\n"
			<< "-l | -no_rotate_log | --no_rotate_log : keine Zeitinfo in Logfile-Namen\n";
}

}

char readKey_Select() {

	fd_set rfds;

	timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 0;
	int max_fd_num = 0;
	int stdin_fd;
	FD_ZERO(&rfds);
	//    FD_SET (js->joystick_fd, &rfds);
	stdin_fd = fileno(stdin);
	FD_SET(stdin_fd, &rfds);

	struct termios to, t;

	tcgetattr(1, &t);
	// saving termios struct
	to = t;
	//toggling canonical mode and echo mode
	t.c_lflag &= ~(ICANON);
	tcsetattr(1, 0, &t);

	char c = -1;

	int retval = select(max_fd_num + 1, &rfds, NULL, NULL, &tv);

	if (FD_ISSET (stdin_fd, &rfds)) // Tastatureingabe;
	{
		char eingabe[100];

		ssize_t r_bytes; // number of bytes read into the buffer
		c = getchar();
//         cout <<"USER INPUT "<< endl;

	}
	tcsetattr(1, 0, &to);

	return c;

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "visionserver");
	Tribots::Vision * the_vision;

	//visual( FL_RGB);

	// recover previous Worldmodel position
	bool auto_restart = true;
	/** rotate_log: decides if time information is appended to logfile
	 * names so logfiles are not lost on multiple calls of the program
	 * (Communicated using configreader, read in AddWriteWorldModel)
	 **/
	bool rotate_log = true;
	bool set_stream_interface = false;
	std::string configfile = ("config_files/robotcontrol.cfg");

	// Command line argument parsing:
	Tribots::ConfigReader cfg_cl(0);
	cfg_cl.add_command_line_shortcut("h", "help", false);
	cfg_cl.add_command_line_shortcut("restart", "restart", false);
	cfg_cl.add_command_line_shortcut("norestart", "norestart", false);
	cfg_cl.add_command_line_shortcut("l", "no_rotate_log", false);
	cfg_cl.add_command_line_shortcut("no_rotate_log", "no_rotate_log", false);
	cfg_cl.add_command_line_shortcut("stream_interface", "stream_interface",
			false);
	cfg_cl.append_from_command_line(argc, argv);

	string s;
	bool b;

	if (cfg_cl.get("norestart", b))
		auto_restart = !b;
	if (cfg_cl.get("restart", b))
		auto_restart = b;
	if (cfg_cl.get("no_rotate_log", b))
		rotate_log = !b;
	if (cfg_cl.get("help", b)) {
		help_text(argv[0]);
		return -1;
	}
	if (cfg_cl.get("stream_interface", b))
		set_stream_interface = true;
	if (cfg_cl.get("ConfigReader::unknown_argument_1", s))
		configfile = s;

#ifdef DEBUGCMDLINEARG
	std::cout << "auto_restart: " << auto_restart << "\n"
	<< "rotate_log: " << rotate_log << "\n"
	<< "config_file: " << configfile << "\n";
#endif

	Tribots::ConfigReader vread(2);
	bool success = vread.append_from_file(configfile.c_str());
	vread.append_from_command_line(argc, argv); // evtl. Parameter durch Kommandozeile ueberschreiben
	vread.set("rotate_log", rotate_log);
	if (set_stream_interface)
		vread.set("user_interface_type", "StreamUserInterface");
	const std::vector<std::string>& conffiles = vread.list_of_sources();
	if (!success) {
		cerr
				<< "Fehler: konnte Konfigurationsdateien nicht vollstaendig lesen\n";
		cerr << "Konfigurationsdatei war: " << configfile << '\n';
		cerr << "Gelesen wurde aus den Dateien:\n";
		for (unsigned int i = 0; i < conffiles.size(); i++)
			cerr << conffiles[i] << '\n';
		return -1;
	}
	Tribots::Journal::the_journal.set_mode(vread);
	for (unsigned int i = 0; i < conffiles.size(); i++)
		JMESSAGE(
				(std::string("tried to read from config file ")+conffiles[i]).c_str());

	std::ofstream lockfile(".robotcontrol_lock");
	if (lockfile) {
		lockfile << getpid() << std::endl;
	}

	unsigned int ui;
	if (vread.get("random_seed", ui) > 0)
		Tribots::random_seed(ui);

	the_vision = new Tribots::Vision(vread);

// ROS STUFF
	ros::Publisher *rossender;

	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<
			opentribot_messages::VisibleObjectList>("VisibleObjects", 1);
	ros::Publisher send_vision_sig = n.advertise<
				opentribot_messages::VisionSignal>("VisionSignal", 1);

	rossender = &chatter_pub;

	int framecounter = 0;

//Fl_RGB_Image flimage = new Fl_RGB_Image(const unsigned char *array, 640,480,4,0);
	const Tribots::Image* tribotsimage;

	Tribots::RGBTuple rgb;

	fl_register_images();

	Fl_Window wMain(640, 480, "Display");
	Fl_Box box(0, 0, 640, 480);
//Fl_JPEG_Image jpg("/tmp/MER_640_480.jpg");

	unsigned char* data;
	data = new unsigned char[640 * 480 * 3];

	wMain.show();

	int window_show = 1;
	int raw_show=0;


	opentribot_messages::VisionSignal vissig;





	while (ros::ok()) {
//usleep(3000000);

		if (window_show) {
			if (raw_show)
				the_vision->request_image_raw(0);
			else
				the_vision->request_image_processed(0);
		}

		the_vision->process_images();
		//cout << "vision get image" << endl;

		if (window_show) {
			tribotsimage = the_vision->get_image(0);
			for (int j = 0; j < 480; j++) {
				for (int i = 0; i < 640; i++) {
					tribotsimage->getPixelRGB(i, j, &rgb);
					data[640 * 3 * j + i * 3 + 0] = rgb.r;
					data[640 * 3 * j + i * 3 + 1] = rgb.g;
					data[640 * 3 * j + i * 3 + 2] = rgb.b;
					//img.draw(i,j);
					//cout <<" | "<< rgb.r;

				}

			}

			the_vision->free_image(0);
			Fl_RGB_Image img(data, 640, 480, 3, 0);
			box.image(img);
			box.redraw();
			Fl::wait(0);
		}

		//unsigned int num_keys = 0;
		char c = readKey_Select();
		int chomp = 1;
		while (chomp > 0)
			chomp = readKey_Select();
		if (c == 'r') raw_show=!raw_show;

		if (c == 'w') {
			if (window_show == 1)
				wMain.hide();
			if (window_show == 0)
				wMain.show();
			window_show = !window_show;
			Fl::wait(0);
		}
		if (c == 'q') {
			ros::shutdown();
		}
		if (c=='j'){
			vissig.param=1;
			send_vision_sig.publish(vissig);
		}
		if (c=='k'){
			vissig.param=2;
			send_vision_sig.publish(vissig);
			cout <<" send stuff"<<endl;
		}
		if (c=='l'){
					vissig.param=3;
					send_vision_sig.publish(vissig);
				}


		opentribot_messages::VisibleObjectList list;

		const Tribots::VisibleObjectList& ol =
				the_vision->get_last_seen_objects(0);

		for (int i = 0; i < ol.objectlist.size(); i++) {

			opentribot_messages::VisibleObject obj;
			obj.posx = ol.objectlist[i].pos.x;
			obj.posy = ol.objectlist[i].pos.y;
			obj.posz = ol.objectlist[i].z;
			obj.width = ol.objectlist[i].width;
			obj.type = ol.objectlist[i].object_type;
			list.objects.push_back(obj);
		}
//ROS_INFO("Sending VisibleOBjectlist");
		framecounter++;
		rossender->publish(list);
		if (framecounter % 60 == 0)
			cout << "."<< endl;
		;

		ros::spinOnce();


	}

	return 0;
}
