#pragma once

#include <functional>
#include <vector>
#include <string>
#include "autonomous/waypoint.hpp"
#include "subsystems/drivetrain.hpp"
#include "pros/rtos.hpp"

/**
 * @file auton_sequence.hpp
 * @brief Fluent autonomous sequence builder.
 *
 * Chains navigation, actions, and delays using a readable builder pattern.
 * All motion steps are synchronous (robot finishes each before starting the next).
 *
 * Example:
 * @code
 *   AutonSequence(drivetrain)
 *       .goTo("START")
 *       .run([]{ intake.spin(127); })
 *       .goTo("SCORE_HIGH")
 *       .run([]{ endeffector.scoreHigh(); pros::delay(500); endeffector.stop(); })
 *       .delay(300)
 *       .goTo("INTAKE_ZONE")
 *       .execute();
 * @endcode
 */
class AutonSequence {
public:
    /**
     * @brief Construct a sequence tied to a drivetrain instance.
     * @param dt  The drivetrain that will execute navigation commands.
     */
    explicit AutonSequence(Drivetrain& dt) : dt_(dt) {}

    // -------------------------------------------------------------------------
    // Navigation
    // -------------------------------------------------------------------------

    /**
     * @brief Navigate to a named waypoint registered in NavMap.
     * @param name  Key used in NavMap::add()
     */
    AutonSequence& goTo(const std::string& name) {
        actions_.push_back([this, name]() {
            dt_.navigateTo(NavMap::get(name));
        });
        return *this;
    }

    /**
     * @brief Navigate to an explicit Waypoint (no registry lookup).
     */
    AutonSequence& goTo(const Waypoint& wp) {
        actions_.push_back([this, wp]() {
            dt_.navigateTo(wp);
        });
        return *this;
    }

    /**
     * @brief Turn to an absolute heading (degrees) without changing position.
     * @param heading   Target heading in degrees.
     * @param timeout   Max time for the turn in ms (default 2000).
     */
    AutonSequence& turnTo(float heading, int timeout = 2000) {
        actions_.push_back([this, heading, timeout]() {
            dt_.get_chassis().turnToHeading(heading, timeout, {}, false);
        });
        return *this;
    }

    // -------------------------------------------------------------------------
    // Actions & timing
    // -------------------------------------------------------------------------

    /**
     * @brief Queue an arbitrary action (lambda, function pointer, etc.).
     * The action runs synchronously — the sequence waits for it to return.
     *
     * @code
     *   .run([]{ intake.spin(127); })
     *   .run([]{ endeffector.scoreHigh(); pros::delay(500); endeffector.stop(); })
     * @endcode
     */
    AutonSequence& run(std::function<void()> fn) {
        actions_.push_back(fn);
        return *this;
    }

    /**
     * @brief Pause execution for a fixed duration.
     * @param ms  Milliseconds to wait.
     */
    AutonSequence& delay(int ms) {
        actions_.push_back([ms]() {
            pros::delay(ms);
        });
        return *this;
    }

    // -------------------------------------------------------------------------
    // Execution
    // -------------------------------------------------------------------------

    /**
     * @brief Run all queued steps in order. Blocks until all complete.
     * Call this at the end of the builder chain.
     */
    void execute() {
        for (auto& action : actions_) {
            action();
        }
    }

private:
    Drivetrain& dt_;
    std::vector<std::function<void()>> actions_;
};
