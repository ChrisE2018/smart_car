/*
 * Copyright (c) 2023 by Christopher Eliot.
 *
 * optional.hpp
 *
 *  Created on: Feb 4, 2023
 *      Author: cre
 */

#pragma once

namespace std
{

template<typename T>
class optional
{
    public:
        optional () noexcept :
                populated(false)
        {
        }

        optional (const T &source) noexcept :
                populated(true), content(source)
        {
        }

        void operator= (T source) noexcept
        {
            content = source;
            populated = true;
        }

        constexpr const T* operator-> () const noexcept
        {
            return &content;
        }

        constexpr const T& operator* () const & noexcept
        {
            return content;
        }

        explicit operator bool () const noexcept
        {
            return populated;
        }

        bool has_value () const noexcept
        {
            return populated;
        }

        constexpr T& value ()
        {
            return content;
        }

        constexpr const T& value ()
        {
            return content;
        }

        T value_or (T default_value)
        {
            if (populated)
            {
                return content;
            }
            else
            {
                return default_value;
            }
        }

        void swap (optional &other) noexcept
        {
            const T &temp = content;
            content = other.content;
            other.content = temp;
        }

        void reset () noexcept
        {
            populated = false;
        }

    private:
        bool populated;
        const T content;
};

}
