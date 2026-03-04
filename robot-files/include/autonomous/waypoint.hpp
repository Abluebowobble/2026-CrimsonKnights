#pragma once

#include <string>
#include <unordered_map>

/**
 * @file waypoint.hpp
 * @brief Named field locations and the NavMap registry.
 *
 * Usage:
 *   // Register a location (typically in initialize() or at the top of auton)
 *   NavMap::add("SCORING_ZONE", {24.0f, 48.0f, 90.0f});
 *
 *   // Retrieve and navigate
 *   drivetrain.navigateTo(NavMap::get("SCORING_ZONE"));
 */

/**
 * @struct Waypoint
 * @brief A named position and heading on the field.
 *
 * Coordinates match LemLib's field convention (inches, 0° = forward).
 * Set timeout to 0 to use the default from DRIVETRAIN_CONSTANTS.
 */
struct Waypoint {
    float x        = 0.0f; ///< Field X position in inches
    float y        = 0.0f; ///< Field Y position in inches
    float heading  = 0.0f; ///< Target heading in degrees (LemLib convention)
    int   timeout  = 3000; ///< Max time allowed to reach this waypoint (ms)
    std::string name = ""; ///< Optional human-readable label (set by NavMap::add)
};

/**
 * @class NavMap
 * @brief Static registry that maps string names to Waypoints.
 *
 * Waypoints are shared across all autonomous routines.
 * Register them once in initialize() or at the start of autonomous().
 *
 * Example:
 * @code
 *   NavMap::add("START",        { 7.8f,  39.4f, 97.3f });
 *   NavMap::add("SCORE_HIGH",   {58.4f,  46.4f,  0.0f });
 *   NavMap::add("INTAKE_ZONE",  {24.0f,  12.0f, 270.0f});
 *
 *   drivetrain.navigateTo(NavMap::get("SCORE_HIGH"));
 * @endcode
 */
class NavMap {
public:
    /**
     * @brief Register a named waypoint.
     * If the name already exists it will be overwritten.
     * @param name  Unique identifier (e.g. "SCORE_HIGH", "START_RED")
     * @param wp    Waypoint data. The name field is automatically set.
     */
    static void add(const std::string& name, Waypoint wp) {
        wp.name = name;
        registry()[name] = wp;
    }

    /**
     * @brief Retrieve a registered waypoint by name.
     * Asserts that the name exists (will cause a runtime error if not found).
     * @param name  The name used when add() was called.
     * @return const Waypoint&
     */
    static const Waypoint& get(const std::string& name) {
        return registry().at(name);
    }

    /**
     * @brief Check whether a name is registered.
     */
    static bool has(const std::string& name) {
        return registry().count(name) > 0;
    }

    /**
     * @brief Remove all registered waypoints.
     */
    static void clear() {
        registry().clear();
    }

private:
    // Meyers singleton — avoids static-init-order issues
    static std::unordered_map<std::string, Waypoint>& registry() {
        static std::unordered_map<std::string, Waypoint> instance;
        return instance;
    }
};
