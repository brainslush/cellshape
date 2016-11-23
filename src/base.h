#include "extIncludes.h"

#ifndef __H_CLASSES_BASE
#define __H_CLASSES_BASE

class visual_base;
class base {
	public:
		base();
		base(std::vector<ofVec2d> iPositions);
		base(std::vector<ofVec2d> iPositions, std::vector<double> iParameters);
		virtual ~base();

		virtual std::vector<ofVec2d>& get_positions();
		virtual std::vector<double>& get_parameters();
		virtual visual_base* get_visualObj();
		virtual unsigned& get_typeID();
		virtual unsigned long long& get_timeStamp();

		virtual void obtain_visualObjs(std::vector<visual_base*>& iVisualObjs);
	protected:
		virtual void update_timeStamp();
		std::vector<ofVec2d> positions; // position of object
		std::vector<double> parameters; // additional parameters of the object
		visual_base* associatedVisualObj; // assignes a visual object
		unsigned long long timeStamp; // timestamp is relevant for update features
		unsigned typeID; // since type_info doesn't work we need to use this
};

/*
 *  Those are all accessible visual elements which can be used to represent the components.
 *  The name might be misleading as the visual element also determines the physical form of the component.
 */

class visual_base {
	public:
		visual_base(unsigned iType, base* iComponent);
		virtual ~visual_base();

		virtual base& get_associatedComponent();
		virtual ofFloatColor& get_color();
		virtual ofFloatColor& get_fillColor();
		virtual unsigned& get_type();
		virtual std::vector<ofVec2d>& get_positions();
		virtual std::vector<double>& get_parameters();

		virtual void set_associatedComponent(base* iComponent);
		virtual void set_color(double iRed,double iGreen, double iBlue, double iAlpha);
		virtual void set_color(double iRed,double iGreen, double iBlue);
		virtual void set_fillColor(double iRed,double iGreen, double iBlue, double iAlpha);
		virtual void set_fillColor(double iRed,double iGreen, double iBlue);
	protected:
		base* associatedComponent;
		ofFloatColor color;
		ofFloatColor fillColor;
		unsigned type;
};

class visual_line:public visual_base {
	public:
		visual_line(base* iComponent);
		virtual ~visual_line();
	protected:
};
class visual_ellipse:public visual_base {
	public:
		visual_ellipse(base* iComponent);
		virtual ~visual_ellipse();
	protected:
};
class visual_rectangle:public visual_base {
	public:
		visual_rectangle(base* iComponent);
		virtual ~visual_rectangle();
	protected:
};
class visual_triangle:public visual_base {
	public:
		visual_triangle(base* iComponent);
		virtual ~visual_triangle();
	protected:
};

#endif
