#ifndef MRMF_UNIQUE_ID_H
#define MRMF_UNIQUE_ID_H

#include <iostream>

namespace mrmf_core
{

// this ID generator is inspired by the descartes trajectory ID generator
// https://github.com/ros-industrial-consortium/descartes

// TODO: add mutex lock

namespace detail
{

template <typename T, class U>
struct IdGenerator;

template <class U>
struct IdGenerator<uint64_t, U>
{
	typedef uint64_t value_type;

	static value_type make_nil()
	{
		return 0;
	}		
	
	static value_type make_id()
	{
		return counter_++;
	}
	
	static bool is_nil(value_type id)
	{
		return id == 0;
	}
	
private:
	static value_type counter_;
};
} // namespace detail


class Robot;

template<typename T>
class RobotID_
{
public:
	typedef T value_type;

	RobotID_(value_type id) : id_(id)
	{
	}

	RobotID_() : id_(detail::IdGenerator<value_type, Robot>::make_nil())
	{
	}

	bool is_nil() const
	{
		return detail::IdGenerator<value_type, Robot>::is_nil(id_);
	}

	value_type value() const
	{
		return id_;
	}

	static RobotID_<value_type> make_nil()
	{
		return RobotID_<value_type>(detail::IdGenerator<value_type, Robot>::make_nil());
	}

	static RobotID_<value_type> make_id()
	{
		return RobotID_<value_type>(detail::IdGenerator<value_type, Robot>::make_id());
	}

private:
	value_type id_;
};

//////////////////////
// Helper Functions //
//////////////////////

template <typename T>
inline bool operator==(RobotID_<T> lhs, RobotID_<T> rhs)
{
  return lhs.value() == rhs.value();
}

template <typename T>
inline bool operator!=(RobotID_<T> lhs, RobotID_<T> rhs)
{
  return !(lhs == rhs);
}

template <typename T>
inline bool operator<(RobotID_<T> lhs, RobotID_<T> rhs)
{
  return lhs.value() < rhs.value();
}

template <typename T>
inline std::ostream& operator<<(std::ostream& os, RobotID_<T> id)
{
  os << "ID" << id.value();
  return os;
}

typedef RobotID_<uint64_t> RobotID;

} // namespace mrmf_core


// inject custom std::hash specialization
template <typename T>
struct std::hash<mrmf_core::RobotID_<T>>
{
	std::size_t operator()(mrmf_core::RobotID_<T> const& id) const noexcept
	{
		return std::hash<T>{}(id.value());
	}
};

#endif // MRMF_UNIQUE_ID_H