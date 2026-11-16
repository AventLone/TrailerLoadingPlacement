#pragma once
#include <Eigen/Core>

#include <deque>
#include <type_traits>
#include <cstddef>

template <typename T, std::size_t WindowSize>
class SlidingMean
{
    static_assert(WindowSize > 0, "WindowSize must be greater than 0");

public:
    SlidingMean()
    {
        setZero(mSum);
    }

    void push(const T& value)
    {
        // Remove the oldest sample when the window is full.
        if (mSamples.size() == WindowSize)
        {
            mSum -= mSamples.front();
            mSamples.pop_front();
        }

        mSamples.push_back(value);
        mSum += value;
    }

    T mean() const
    {
        if (mSamples.empty())
        {
            return mSum;
        }

        return mSum / static_cast<float>(mSamples.size());
    }

    void reset()
    {
        mSamples.clear();
        setZero(mSum);
    }

    std::size_t size() const noexcept
    {
        return mSamples.size();
    }

private:
    static void setZero(T& value)
    {
        if constexpr (std::is_arithmetic_v<T>)
        {
            value = T{0};
        }
        else
        {
            value.setZero();
        }
    }

    std::deque<T> mSamples;
    T mSum;
};

template<int WINDOW_SIZE>
struct HistoryBuffer
{
    // static constexpr std::size_t WINDOW_SIZE = 200;

    Eigen::Vector3f trailer_pose() const noexcept
    {
        return trailer_pose_history.mean();
    }

    Eigen::Vector3f trailer_dimensions() const noexcept
    {
        return trailer_dimensions_history.mean();
    }

    Eigen::Vector3f ramp_position() const noexcept
    {
        return ramp_position_history.mean();
    }

    Eigen::Vector2f ramp_size() const noexcept
    {
        return ramp_size_history.mean();
    }

    float ramp_tilt() const noexcept
    {
        return ramp_tilt_history.mean();
    }

    void reset()
    {
        trailer_pose_history.reset();
        trailer_dimensions_history.reset();
        ramp_position_history.reset();
        ramp_size_history.reset();
        ramp_tilt_history.reset();
    }

    void pushTrailerPose(const Eigen::Vector3f& value)
    {
        trailer_pose_history.push(value);
    }

    void pushTrailerDimensions(const Eigen::Vector3f& value)
    {
        trailer_dimensions_history.push(value);
    }

    void pushRampPosition(const Eigen::Vector3f& value)
    {
        ramp_position_history.push(value);
    }

    void pushRampSize(const Eigen::Vector2f& value)
    {
        ramp_size_history.push(value);
    }

    void pushRampTilt(const float value)
    {
        ramp_tilt_history.push(value);
    }

private:
    SlidingMean<Eigen::Vector3f, WINDOW_SIZE> trailer_pose_history;
    SlidingMean<Eigen::Vector3f, WINDOW_SIZE> trailer_dimensions_history;
    SlidingMean<Eigen::Vector3f, WINDOW_SIZE> ramp_position_history;
    SlidingMean<Eigen::Vector2f, WINDOW_SIZE> ramp_size_history;
    SlidingMean<float, WINDOW_SIZE> ramp_tilt_history;
};