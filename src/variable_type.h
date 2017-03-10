#include "extIncludes.h"

class variable_base {
public:
	variable_base();
	~variable_base();
};

template <class VT>
class variable_type: public variable_base {
public:
	variable_type();
	variable_type(std::string iName, std::string iLabel);
	~variable_type();
	void set_updated(bool iUpdated);
	bool& isUpdated();
	void set_initValue(VT iInitValue);
	void set_value(VT iValue);
	VT& get_initValue();
	VT& get_value();
	VT& get_lastValue();
	std::string& get_name();
	std::string& get_label();
	VT operator+ (variable_type& rhs);
	VT operator- (variable_type& rhs);
	VT operator* (variable_type& rhs);
	VT operator/ (variable_type& rhs);
	variable_type<VT>& operator= (variable_type& iValue);
	variable_type<VT>& operator= (const VT iValue);
	variable_type<VT>& operator+= (variable_type& iValue);
	variable_type<VT>& operator+= (const VT iValue);
	variable_type<VT>& operator-= (variable_type& iValue);
	variable_type<VT>& operator-= (const VT iValue);
	variable_type<VT>& operator*= (variable_type& iValue);
	variable_type<VT>& operator*= (const VT iValue);
	variable_type<VT>& operator/= (variable_type& iValue);
	variable_type<VT>& operator/= (const VT iValue);
private:
	VT initValue;
	VT lastValue;
	VT value;
	const std::string name;
	const std::string label;
	bool updated;
};

/* Defines of templates */

template<class VT>
inline variable_type<VT>::variable_type(): name(""), label("") {
	updated = false;
}
template <class VT>
inline variable_type<VT>::variable_type(std::string iName, std::string iLabel): name(iName), label(iLabel){
	updated = false;
}
template<class VT> inline variable_type<VT>::~variable_type() {}
template<class VT> inline void variable_type<VT>::set_initValue(VT iInitValue) {initValue = iInitValue;}
template<class VT> inline void variable_type<VT>::set_value(VT iValue) {
	lastValue = value;
	value = iValue;
}
template<class VT> inline void variable_type<VT>::set_updated(bool iUpdated) {updated = iUpdated;}
template<class VT> inline bool& variable_type<VT>::isUpdated() {return updated;}
template<class VT> inline VT& variable_type<VT>::get_initValue() {return initValue;}
template<class VT> inline VT& variable_type<VT>::get_value() {return value;}
template<class VT> inline VT& variable_type<VT>::get_lastValue() {return lastValue;}
template<class VT> inline std::string& variable_type<VT>::get_name() {return name;}
template<class VT> inline std::string& variable_type<VT>::get_label() {return label;}

template<class VT>
inline VT variable_type<VT>::operator+(variable_type& rhs) {
	VT temp;
	temp = lhs.get_value() + rhs.get_value();
	return temp;
}
template<class VT>
inline VT variable_type<VT>::operator-(variable_type& rhs) {
	VT temp;
	temp = lhs.get_value() - rhs.get_value();
	return temp;
}
template<class VT>
inline VT variable_type<VT>::operator*(variable_type & rhs) {
	return VT();
}
template<class VT>
inline VT variable_type<VT>::operator/(variable_type & rhs) {
	return VT();
}
template<class VT>
inline variable_type<VT>& variable_type<VT>::operator=(variable_type& iObj) {
	this->set_value(iObj.get_value());
	return *this;
}
template<class VT>
inline variable_type<VT>& variable_type<VT>::operator=(const VT iValue) {
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