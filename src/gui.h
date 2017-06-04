/*
 * gui.h
 *
 *  Created on: Jun 1, 2017
 *      Author: siegbahn
 */

#include "extIncludes.h"
#include "ofxGui.h"

#ifndef SRC_GUI_H_
#define SRC_GUI_H_

namespace mygui {
  typedef ofxSlider<unsigned long long> ofxULLongSlider;
  typedef ofxSlider<double> ofxDoubleSlider;
  typedef ofxSlider<long long> ofxLongLongSlider;
  using ofxControlTypes = boost::variant<
      ofxLabel*,
      ofxToggle*,
      ofxFloatSlider*,
      ofxDoubleSlider*,
      ofxULLongSlider*,
      ofxLongLongSlider*,
      ofxIntSlider*
  >;

  class setting_base {
  public:
      setting_base(){};
      virtual ~setting_base(){};
      virtual ofxBaseGui* setup(){return nullptr;};
      virtual void update(){};
  protected:
  };
  template<typename T> class setting: public setting_base {
  public:
      setting (
          std::string& iLabel,
          bool& iUpdatePerFrame,
          float iWidth,
          float iHeight,
          T iValue
      ) :
          label(iLabel),
          updatePerFrame(iUpdatePerFrame),
          value(iValue),
          width(iWidth),
          height(iHeight)
      {
          control = new ofxToggle;
          minValue = false;
          maxValue = false;
      }
      setting(
          std::string& iLabel,
          bool& iUpdatePerFrame,
          float iWidth,
          float iHeight,
          T iValue,
          T iMinValue,
          T iMaxValue
      ) :
          label(iLabel),
          updatePerFrame(iUpdatePerFrame),
          value(iValue),
          minValue(iMinValue),
          maxValue(iMaxValue),
          width(iWidth),
          height(iHeight)
      {
          switch(typeid(T)) {
              case (typeid(bool)) : {
                  control = new ofxToggle;
              }
              break;
              case (typeid(double)) : {
                  control = new ofxDoubleSlider;
              }
              break;
              case (typeid(float)) : {
                  control = new ofxFloatSlider;
              }
              break;
              case (typeid(unsigned long long)) : {
                  control = new ofxULLongSlider;
              }
              break;
              case (typeid(long long)) : {
                  control = new ofxLongLongSlider;
              }
              break;
              case (typeid(int)) : {
                  control = new ofxIntSlider;
              }
              break;
              default : {
                  label = "ERROR 1";
                  control = new ofxLabel;
              }
          }
      }
      virtual ~setting() {

      }
      virtual std::string& get_name() {
          return label;
      }
      virtual T* get_pointer() {
          return &value;
      }
      virtual T& get_value() {
          return value;
      }
      virtual ofxBaseGui* setup() {
          if(control.type() == typeid(ofxToggle*)) {
              ofxToggle* t = boost::get<ofxToggle*>(control);
              return t->setup(label,value,width,height);
          } else if(control.type() == typeid(ofxDoubleSlider*)) {
              ofxDoubleSlider* t = boost::get<ofxDoubleSlider*>(control);
              return t->setup(label,value,minValue,maxValue,width,height);
          } else if(control.type() == typeid(ofxFloatSlider*)) {
              ofxFloatSlider* t = boost::get<ofxFloatSlider*>(control);
              return t->setup(label,value,minValue,maxValue,width,height);
          } else if(control.type() == typeid(ofxLongLongSlider*)) {
              ofxLongLongSlider* t = boost::get<ofxLongLongSlider*>(control);
              return t->setup(label,value,minValue,maxValue,width,height);
          } else if(control.type() == typeid(ofxULLongSlider*)) {
              ofxULLongSlider* t = boost::get<ofxULLongSlider*>(control);
              return t->setup(label,value,minValue,maxValue,width,height);
          } else if(control.type() == typeid(ofxIntSlider*)) {
              ofxIntSlider* t = boost::get<ofxIntSlider*>(control);
              return t->setup(label,value,minValue,maxValue,width,height);
          } else {
              label = "ERROR 2";
              control = new ofxLabel;
              ofxLabel* t = boost::get<ofxLabel*>(control);
              return t->setup(label,width,height);
          }
      }
      virtual void update() {
          if(updatePerFrame) {
              if(control.type() == typeid(ofxToggle*)) {
                  ofxToggle* t = boost::get<ofxToggle*>(control);
                  value = *t;
              } else if (control.type() == typeid(ofxDoubleSlider*)) {
                  ofxDoubleSlider* t = boost::get<ofxDoubleSlider*>(control);
                  value = *t;
              } else if (control.type() == typeid(ofxFloatSlider*)) {
                  ofxFloatSlider* t = boost::get<ofxFloatSlider*>(control);
                  value = *t;
              } else if (control.type() == typeid(ofxLongLongSlider*)) {
                  ofxLongLongSlider* t = boost::get<ofxLongLongSlider*>(control);
                  value = *t;
              } else if (control.type() == typeid(ofxULLongSlider*)) {
                  ofxULLongSlider* t = boost::get<ofxULLongSlider*>(control);
                  value = *t;
              } else if (control.type() == typeid(ofxIntSlider*)) {
                  ofxIntSlider* t = boost::get<ofxIntSlider*>(control);
                  value = *t;
              }
          }
      }
  protected:
      std::string label;
      bool updatePerFrame;
      T value;
      T minValue;
      T maxValue;
      float width;
      float height;
      ofxControlTypes control;
  };

  class group {
  public:
      group();
      group(std::string iName);
      virtual ~group();
      virtual void setup();
      virtual void draw();
      virtual void update();
      template <typename T,typename... A>
      T& register_setting(
          std::string iLabel,
          bool iUpdatePerFrame,
          float iWidth,
          float iHeight,
          A... iArgs
      ) {
          setting<T>* newSetting = new setting<T>(
              iLabel,
              iUpdatePerFrame,
              iWidth,
              iHeight,
              iArgs...
          );
          settings.insert(newSetting);
          return newSetting->get_value();
      }
      template <typename T,typename... A>
      T* register_setting(
          std::string iLabel,
          bool iUpdatePerFrame,
          A... iArgs
      ) {
          setting<T>* newSetting = new setting<T>(
              iLabel,
              iUpdatePerFrame,
              200,
              18,
              iArgs...
          );
          settings.insert(newSetting);
          return newSetting->get_pointer();
      }
  protected:
      std::string name;
      std::set<setting_base*> settings;
      ofxPanel panel;
  };


  class gui {
  public:
      gui();
      virtual ~gui();
      virtual void setup();
      virtual void draw();
      virtual void update();
      group* register_group();
      group* register_group(std::string iName);
      virtual void unregister_group(group* iGroup);
  protected:
      std::set<group*> groups;
      ofxPanel mainPanel;
      ofxLabel mainLabel;
      ofxLabel FPS;
      ofxButton playPause;
      ofxButton stopReset;
  };
}


#endif /* SRC_GUI_H_ */
