#ifndef __H_CLASSES_BASE
#define __H_CLASSES_BASE

class visual_base;
class base {
	public:
		base();
		virtual ~base();

		virtual std::vector<ofVec2d>& get_positions();
		virtual std::vector<double>& get_parameters();
		virtual visual_base* get_visualObj();

		virtual void obtain_visualObjs(std::vector<visual_base*>& iVisualObjs);
	protected:
		std::vector<ofVec2d> positions; // position of object
		std::vector<double> parameters; // additional parameters of the object
		visual_base* associatedVisualObj; // assignes a visual object
};

#endif
