/*
    Copyright (C) 2003-2004  Kevin Thayer <nufan_wfk at yahoo dot com>

    Cleanup and VBI and audio in/out options, introduction in v4l-dvb,
    support for most new APIs since 2006.
    Copyright (C) 2004, 2006, 2007  Hans Verkuil <hverkuil@xs4all.nl>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

//#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <math.h>

#include <linux/videodev2.h>

#include <list>
#include <vector>
#include <map>
#include <string>
#include <algorithm>



using namespace std;





class V4l2_Controls{
public :
	char *value, *subs;
	int i;

	int fd ;


	/* command args */
	int ch;
	const char *device;	/* -d device */
	struct v4l2_capability vcap;	/* list_cap */
	char short_options[26 * 2 * 2 + 1];
	int idx;


V4l2_Controls(){
fd=-1;
device="/dev/video0";
idx=0;
};

~V4l2_Controls(){
close(fd);
}




/* Short option list

   Please keep in alphabetical order.
   That makes it easier to see which short options are still free.

   In general the lower case is used to set something and the upper
   case is used to retrieve a setting. */
enum Option {
	OptGetCtrl = 'C',
	OptSetCtrl = 'c',
	OptSetDevice = 'd',
	OptHelp = 'h',
	OptListCtrls = 'l',
	OptListCtrlsMenus = 'L',
	OptLast = 256
};

char options[OptLast];

int app_result;
int verbose;

unsigned capabilities;

typedef std::vector<struct v4l2_ext_control> ctrl_list;
ctrl_list user_ctrls;
ctrl_list mpeg_ctrls;

typedef std::map<std::string, unsigned> ctrl_strmap;
ctrl_strmap ctrl_str2id;
typedef std::map<unsigned, std::string> ctrl_idmap;
ctrl_idmap ctrl_id2str;

typedef std::list<std::string> ctrl_get_list;
ctrl_get_list get_ctrls;

typedef std::map<std::string,std::string> ctrl_set_map;
 ctrl_set_map set_ctrls;

typedef std::vector<std::string> dev_vec;
typedef std::map<std::string, std::string> dev_map;

typedef struct {
	unsigned flag;
	const char *str;
} flag_def;





std::string flags2s(unsigned val, const flag_def *def)
  {
          std::string s;
  
          while (def->flag) {
                  if (val & def->flag) {
                          if (s.length()) s += " ";
                          s += def->str;
                  }
                  def++;
          }
          return s;
  }



 std::string name2var(unsigned char *name)
{
	std::string s;
	int add_underscore = 0;

	while (*name) {
		if (isalnum(*name)) {
			if (add_underscore)
				s += '_';
			add_underscore = 0;
			s += std::string(1, tolower(*name));
		}
		else if (s.length()) add_underscore = 1;
		name++;
	}
	return s;
}


 void print_qctrl(int fd, struct v4l2_queryctrl *queryctrl,
		struct v4l2_ext_control *ctrl, int show_menus)
{
	struct v4l2_querymenu qmenu = { 0 };
	std::string s = name2var(queryctrl->name);
	int i;

	qmenu.id = queryctrl->id;
	switch (queryctrl->type) {
	case V4L2_CTRL_TYPE_INTEGER:
		printf("%31s (int)  : min=%d max=%d step=%d default=%d value=%d",
				s.c_str(),
				queryctrl->minimum, queryctrl->maximum,
				queryctrl->step, queryctrl->default_value,
				ctrl->value);
		break;
	case V4L2_CTRL_TYPE_INTEGER64:
		printf("%31s (int64): value=%lld", s.c_str(), ctrl->value64);
		break;
	case V4L2_CTRL_TYPE_BOOLEAN:
		printf("%31s (bool) : default=%d value=%d",
				s.c_str(),
				queryctrl->default_value, ctrl->value);
		break;
	case V4L2_CTRL_TYPE_MENU:
		printf("%31s (menu) : min=%d max=%d default=%d value=%d",
				s.c_str(),
				queryctrl->minimum, queryctrl->maximum,
				queryctrl->default_value, ctrl->value);
		break;
	case V4L2_CTRL_TYPE_BUTTON:
		printf("%31s (button)\n", s.c_str());
		break;
	default: break;
	}
	if (queryctrl->flags) {
		const flag_def def[] = {
			{ V4L2_CTRL_FLAG_GRABBED,    "grabbed" },
			{ V4L2_CTRL_FLAG_READ_ONLY,  "read-only" },
			{ V4L2_CTRL_FLAG_UPDATE,     "update" },
			{ V4L2_CTRL_FLAG_INACTIVE,   "inactive" },
			{ V4L2_CTRL_FLAG_SLIDER,     "slider" },
			{ 0, NULL }
		};
		printf(" flags=%s", flags2s(queryctrl->flags, def).c_str());
	}
	printf("\n");
	if (queryctrl->type == V4L2_CTRL_TYPE_MENU && show_menus) {
		for (i = 0; i <= queryctrl->maximum; i++) {
			qmenu.index = i;
			if (ioctl(fd, VIDIOC_QUERYMENU, &qmenu))
				continue;
			printf("\t\t\t\t%d: %s\n", i, qmenu.name);
		}
	}
}

int print_control(int fd, struct v4l2_queryctrl &qctrl, int show_menus)
{

	struct v4l2_control ctrl = { 0 };
	struct v4l2_ext_control ext_ctrl = { 0 };
	struct v4l2_ext_controls ctrls = { 0 };

	if (qctrl.flags & V4L2_CTRL_FLAG_DISABLED)
		return 1;
	if (qctrl.type == V4L2_CTRL_TYPE_CTRL_CLASS) {
		printf("\n%s\n\n", qctrl.name);
		return 1;
	}
	ext_ctrl.id = qctrl.id;
	ctrls.ctrl_class = V4L2_CTRL_ID2CLASS(qctrl.id);
	ctrls.count = 1;
	ctrls.controls = &ext_ctrl;
	if (V4L2_CTRL_ID2CLASS(qctrl.id) != V4L2_CTRL_CLASS_USER &&
	    qctrl.id < V4L2_CID_PRIVATE_BASE) {
		if (ioctl(fd, VIDIOC_G_EXT_CTRLS, &ctrls)) {
			printf("error %d getting ext_ctrl %s\n",
					errno, qctrl.name);
			return 0;
		}
	}
	else {
		ctrl.id = qctrl.id;
		if (ioctl(fd, VIDIOC_G_CTRL, &ctrl)) {
			printf("error %d getting ctrl %s\n",
					errno, qctrl.name);
			return 0;
		}
		ext_ctrl.value = ctrl.value;
	}
	print_qctrl(fd, &qctrl, &ext_ctrl, show_menus);
	return 1;
}

 void list_controls(int fd, int show_menus)
{
	struct v4l2_queryctrl qctrl = { V4L2_CTRL_FLAG_NEXT_CTRL };
	int id;

	while (ioctl(fd, VIDIOC_QUERYCTRL, &qctrl) == 0) {
			print_control(fd, qctrl, show_menus);
		qctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
	}
	if (qctrl.id != V4L2_CTRL_FLAG_NEXT_CTRL)
		return;
	for (id = V4L2_CID_USER_BASE; id < V4L2_CID_LASTP1; id++) {
		qctrl.id = id;
		if (ioctl(fd, VIDIOC_QUERYCTRL, &qctrl) == 0)
			print_control(fd, qctrl, show_menus);
	}
	for (qctrl.id = V4L2_CID_PRIVATE_BASE;
			ioctl(fd, VIDIOC_QUERYCTRL, &qctrl) == 0; qctrl.id++) {
		print_control(fd, qctrl, show_menus);
	}
}

 void find_controls(int fd)
{
	printf("finding controls\n");
	struct v4l2_queryctrl qctrl = { V4L2_CTRL_FLAG_NEXT_CTRL };
	int id;

	while (ioctl(fd, VIDIOC_QUERYCTRL, &qctrl) == 0) {
		if (qctrl.type != V4L2_CTRL_TYPE_CTRL_CLASS &&
		    !(qctrl.flags & V4L2_CTRL_FLAG_DISABLED)) {
			ctrl_str2id[name2var(qctrl.name)] = qctrl.id;
			ctrl_id2str[qctrl.id] = name2var(qctrl.name);
		}
		qctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
	}
	if (qctrl.id != V4L2_CTRL_FLAG_NEXT_CTRL)
		return;
	for (id = V4L2_CID_USER_BASE; id < V4L2_CID_LASTP1; id++) {
		qctrl.id = id;
		if (ioctl(fd, VIDIOC_QUERYCTRL, &qctrl) == 0 &&
		    !(qctrl.flags & V4L2_CTRL_FLAG_DISABLED)) {
			ctrl_str2id[name2var(qctrl.name)] = qctrl.id;
			ctrl_id2str[qctrl.id] = name2var(qctrl.name);
		}
	}
	for (qctrl.id = V4L2_CID_PRIVATE_BASE;
			ioctl(fd, VIDIOC_QUERYCTRL, &qctrl) == 0; qctrl.id++) {
		if (!(qctrl.flags & V4L2_CTRL_FLAG_DISABLED)) {
			ctrl_str2id[name2var(qctrl.name)] = qctrl.id;
			ctrl_id2str[qctrl.id] = name2var(qctrl.name);
		}
	}
}



int doioctl(int fd, int request, void *parm, const char *name)
{
	int retVal = ioctl(fd, request, parm);

	if (retVal < 0) {
		app_result = -1;
	}
	if (retVal < 0)
		printf("%s: failed: %s\n", name, strerror(errno));
	else if (verbose)
		printf("%s: ok\n", name);

	return retVal;
}




 void parse_next_subopt(char **subs, char **value)
{
	static char *const subopts[] = {
	    NULL
	};
	int opt = getsubopt(subs, subopts, value);

	if (value == NULL) {
		fprintf(stderr, "No value given to suboption <%s>\n",
				subopts[opt]);
		exit(1);
	}
}


int get_control(string name,int value){
	/*	subs = optarg;
		while (*subs != '\0') {
			parse_next_subopt(&subs, &value);
			if (strchr(value, '=')) {
			    usage();
			    exit(1);
			}
			else {
			    get_ctrls.push_back(value);
			}
		}
*/


}
int set_control(string name,int value){

		//struct v4l2_ext_controls ctrls = { 0 };
		//struct v4l2_ext_control ctrl = { 0 };
		struct v4l2_control ctrl;
		ctrl.id = ctrl_str2id[name];
		ctrl.value = value;
/*		if (V4L2_CTRL_ID2CLASS(ctrl.id) == V4L2_CTRL_CLASS_MPEG)
				mpeg_ctrls.push_back(ctrl);
			else
				user_ctrls.push_back(ctrl);
		}*/

			if (doioctl(fd, VIDIOC_S_CTRL, &ctrl, "VIDIOC_S_CTRL")) {
				fprintf(stderr, "%s: %s\n",
					ctrl_id2str[ctrl.id].c_str(),
					strerror(errno));
			}
		
/*		if (mpeg_ctrls.size()) {
			ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;
			ctrls.count = mpeg_ctrls.size();
			ctrls.controls = &mpeg_ctrls[0];
			if (doioctl(fd, VIDIOC_S_EXT_CTRLS, &ctrls, "VIDIOC_S_EXT_CTRLS")) {
				if (ctrls.error_idx >= ctrls.count) {
					fprintf(stderr, "Error setting MPEG controls: %s\n",
						strerror(errno));
				}
				else {
					fprintf(stderr, "%s: %s\n",
						ctrl_id2str[mpeg_ctrls[ctrls.error_idx].id].c_str(),
						strerror(errno));
				}
			}
		}
*/

}

int set_device(const char *name){
	device=name;

	if ((fd = open(device, O_RDWR)) < 0) {
		fprintf(stderr, "Failed to open %s: %s\n", device,
			strerror(errno));
		exit(1);
	}
	capabilities = vcap.capabilities;
	find_controls(fd);
	for (ctrl_get_list::iterator iter = get_ctrls.begin(); iter != get_ctrls.end(); ++iter) {
	    if (ctrl_str2id.find(*iter) == ctrl_str2id.end()) {
		fprintf(stderr, "unknown control '%s'\n", (*iter).c_str());
		exit(1);
	    }
	}
	for (ctrl_set_map::iterator iter = set_ctrls.begin(); iter != set_ctrls.end(); ++iter) {
	    if (ctrl_str2id.find(iter->first) == ctrl_str2id.end()) {
		fprintf(stderr, "unknown control '%s'\n", iter->first.c_str());
		exit(1);
	    }
	}

		list_controls(fd, 0);

return 0;
}



};


int main(int argc, char **argv)
{




	V4l2_Controls controls;

	controls.set_device("/dev/video0");



	controls.set_control("exposure",111);

	/* Information Opts */

/*
	if (options[OptGetCtrl] && !get_ctrls.empty()) {
		struct v4l2_ext_controls ctrls = { 0 };

		mpeg_ctrls.clear();
		user_ctrls.clear();
		for (ctrl_get_list::iterator iter = get_ctrls.begin();
				iter != get_ctrls.end(); ++iter) {
			struct v4l2_ext_control ctrl = { 0 };

			ctrl.id = ctrl_str2id[*iter];
			if (V4L2_CTRL_ID2CLASS(ctrl.id) == V4L2_CTRL_CLASS_MPEG)
				mpeg_ctrls.push_back(ctrl);
			else
				user_ctrls.push_back(ctrl);
		}
		for (unsigned i = 0; i < user_ctrls.size(); i++) {
			struct v4l2_control ctrl;

			ctrl.id = user_ctrls[i].id;
			doioctl(fd, VIDIOC_G_CTRL, &ctrl, "VIDIOC_G_CTRL");
			printf("%s: %d\n", ctrl_id2str[ctrl.id].c_str(), ctrl.value);
		}
		if (mpeg_ctrls.size()) {
			ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;
			ctrls.count = mpeg_ctrls.size();
			ctrls.controls = &mpeg_ctrls[0];
			doioctl(fd, VIDIOC_G_EXT_CTRLS, &ctrls, "VIDIOC_G_EXT_CTRLS");
			for (unsigned i = 0; i < mpeg_ctrls.size(); i++) {
				struct v4l2_ext_control ctrl = mpeg_ctrls[i];

				printf("%s: %d\n", ctrl_id2str[ctrl.id].c_str(), ctrl.value);
			}
		}
*/

	exit(0);
}
