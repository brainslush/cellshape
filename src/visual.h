#ifndef __H_CLASSES_VISUAL
#define __H_CLASSES_VISUAL

/*
 *  Those are all accessible visual elements which can be used to represent the components.
 *  The name might be misleading as the visual element also determines the physical form of the component.
 */

class visual_base {
	public:
		//visual_base();
		visual_base(unsigned iType, components_base* iComponent);
		virtual ~visual_base();

		virtual components_base& get_associatedComponent();
		virtual ofFloatColor& get_color();
		virtual ofFloatColor& get_fillColor();
		virtual unsigned& get_type();
		virtual std::vector<ofVec2d>& get_positions();
		virtual std::vector<double>& get_parameters();

		virtual void set_associatedComponent(components_base* iComponent);
		virtual void set_color(double iRed,double iGreen, double iBlue, double iAlpha);
		virtual void set_color(double iRed,double iGreen, double iBlue);
		virtual void set_fillColor(double iRed,double iGreen, double iBlue, double iAlpha);
		virtual void set_fillColor(double iRed,double iGreen, double iBlue);
	protected:
		components_base* associatedComponent;
		ofFloatColor color;
		ofFloatColor fillColor;
		unsigned type;
};

class visual_line:public visual_base {
	public:
		visual_line(components_base* iComponent);
		virtual ~visual_line();
	protected:
};
class visual_ellipse:public visual_base {
	public:
		visual_ellipse(components_base* iComponent);
		virtual ~visual_ellipse();
	protected:
};
class visual_rectangle:public visual_base {
	public:
		visual_rectangle(components_base* iComponent);
		virtual ~visual_rectangle();
	protected:
};
class visual_triangle:public visual_base {
	public:
		visual_triangle(components_base* iComponent);
		virtual ~visual_triangle();
	protected:
};

#endif
