//#include "extIncludes.h"
#include <vector>
#include <string>

template <class VT>
class variable_type {
public:
    variable_type();
    variable_type(std::string iName, std::string iLabel);
    ~variable_type();
    void set_updated(bool iUpdated);
    void set_initValue(VT iInitValue);
    void set_value(VT iValue);
    void set_lastValue(VT iLastValue);
    bool& isUpdated();
    VT& get_initValue();
    VT& get_value();
    VT& get_lastValue();
    std::string& get_name();
    std::string& get_label();
    VT& get_mean();
    VT& get_variance();
    std::vector<VT>& get_timeLine();
    void start_timeLine();
    void stop_timeLine();
    void reset_timeLine();
    VT operator+ (variable_type<VT>& rhs);
    VT operator- (variable_type<VT>& rhs);
    VT operator* (variable_type<VT>& rhs);
    VT operator/ (variable_type<VT>& rhs);
    variable_type<VT>& operator= (VT& rhs);
    VT operator= (variable_type<VT>& rhs);
    variable_type<VT>& operator+= (variable_type<VT>& rhs);
    variable_type<VT>& operator+= (VT& rhs);
    variable_type<VT>& operator-= (variable_type<VT>& rhs);
    variable_type<VT>& operator-= (VT& rhs);
    variable_type<VT>& operator*= (variable_type<VT>& rhs);
    variable_type<VT>& operator*= (VT& rhs);
    variable_type<VT>& operator/= (variable_type<VT>& rhs);
    variable_type<VT>& operator/= (VT& rhs);
private:
    VT initValue;
    VT lastValue;
    VT value;
    VT mean;
    VT variance;
    std::vector<VT> timeLine;
    const std::string name;
    const std::string label;
    bool updated;
    bool meanUpdated;
    bool varianceUpdated;
};

/* Define constructor and destructor*/
template<class VT> inline variable_type<VT>::variable_type():
    name(""),
    label("")
{
    mean = 0;
    variance = 0;
    updated = false;
    meanUpdated = false;
    varianceUpdated = false;
}
template <class VT> inline variable_type<VT>::variable_type(
    std::string iName,
    std::string iLabel
):
    name(iName),
    label(iLabel)
{
    mean = 0;
    variance = 0;
    updated = false;
    meanUpdated = false;
    varianceUpdated = false;
}
template<class VT> inline variable_type<VT>::~variable_type() {}

/*********************/
/* Define set values */
template<class VT> inline void variable_type<VT>::set_initValue(VT iInitValue) {
    initValue = iInitValue;
}
template<class VT> inline void variable_type<VT>::set_value(VT iValue) {
    lastValue = value;
    value = iValue;
}
template<class VT> inline void variable_type<VT>::set_updated(bool iUpdated) {
    updated = iUpdated;
}
template<class VT> inline bool& variable_type<VT>::isUpdated() {
    return updated;
}

/********************/
/* Define get values*/
template<class VT> inline VT& variable_type<VT>::get_initValue() {
    return initValue;
}
template<class VT> inline VT& variable_type<VT>::get_value() {
    return value;
}
template<class VT> inline VT& variable_type<VT>::get_lastValue() {
    return lastValue;
}
template<class VT> inline std::string& variable_type<VT>::get_name() {
    return name;
}
template<class VT> inline std::string& variable_type<VT>::get_label() {
    return label;
}
template<class VT> inline VT& variable_type<VT>::get_mean() {
    if (!meanUpdated) {
        mean = std::accumulate(timeLine.front(),timeLine.end(),0) / (double) timeLine.size();
        meanUpdated = true;
    }
    return mean;
}
template<class VT> inline VT& variable_type<VT>::get_variance() {
    if (!varianceUpdated) {
        variance = 0;
        for (unsigned long long i = 0; i < timeLine.size(); i++) {
            variance += (timeLine.at(i) - get_mean())*(timeLine.at(i) - get_mean());
        }
        variance /= (double) timeLine.size();
        varianceUpdated = true;
    }
    return variance;
}
template<class VT> inline std::vector<VT>& variable_type<VT>::get_timeLine() {
    return timeLine;
}

/******************/
/*Define operators*/
template<class VT> inline VT variable_type<VT>::operator+(variable_type<VT>& rhs) {
    VT temp = *this + rhs.get_value();
    return temp;
}
template<class VT> inline VT variable_type<VT>::operator-(variable_type<VT>& rhs) {
    VT temp = *this - rhs.get_value();
    return temp;
}
template<class VT> inline VT variable_type<VT>::operator*(variable_type<VT>& rhs) {
    VT temp = *this * rhs.get_value();
    return VT();
}
template<class VT> inline VT variable_type<VT>::operator/(variable_type<VT>& rhs) {
    VT temp = *this / rhs.get_value();
    return VT();
}
template<class VT> inline variable_type<VT>& variable_type<VT>::operator=(variable_type<VT>& rhs) {
    this->set_lastValue(this->get_lastValue());
    this->set_value(rhs.get_value());
    return *this;
}
template<class VT> inline variable_type<VT>& variable_type<VT>::operator=(VT& iValue) {
    this->set_value(iValue);
    return *this;
}
template<class VT>
inline variable_type<VT>& variable_type<VT>::operator+=(variable_type& iObj) {
	this->set_value(value + iObj.get_value());
	return *this;
}
template<class VT>
inline variable_type<VT>& variable_type<VT>::operator+=(const VT iValue) {
	this->set_value(value + iValue);
	return *this;
}
template<class VT>
inline variable_type<VT>& variable_type<VT>::operator-=(variable_type& iObj) {
	this->set_value(value - iObj.get_value());
	return *this;
}
template<class VT>
inline variable_type<VT>& variable_type<VT>::operator-=(const VT iValue) {
	this->set_value(value - iValue);
	return *this;
}
template<class VT>
inline variable_type<VT>& variable_type<VT>::operator*=(variable_type & iValue) {
	// TODO: insert return statement here
}
template<class VT>
inline variable_type<VT>& variable_type<VT>::operator*=(const VT iValue) {
	// TODO: insert return statement here
}
template<class VT>
inline variable_type<VT>& variable_type<VT>::operator/=(variable_type & iValue) {
	// TODO: insert return statement here
}
template<class VT>
inline variable_type<VT>& variable_type<VT>::operator/=(const VT iValue) {
	// TODO: insert return statement here
}
