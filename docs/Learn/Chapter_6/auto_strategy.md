# **FRC Autonomous Programming - Strategy and Planning**

## Overview

This guide teaches you the strategic foundation and planning principles for designing high-performance autonomous routines for FRC robots, covering game analysis, alliance coordination, and decision-making frameworks.

> **Note:** This guide utilizes many Command-Based Programming concepts. For a solid foundation, please review the [Command-Based Theory](../command_based_theory.md) training before proceeding.

**What you'll build:** A strategic framework for autonomous planning that maximizes competitive advantage and adapts to different match scenarios.

---

## 1. Why Autonomous Matters?

For competitive FRC teams, autonomous mode is often the difference between winning and losing matches.

**Simple Mobility?** This gets your robot moving but doesn't maximize scoring potential.

```java
// Basic mobility - just drive forward
public Command getMobilityAuto() {
    return new DriveForward(driveSubsystem, 3.0); // Drive 3 meters
}
```
This approach works but leaves significant points on the table and provides no strategic advantage.

**Static Autonomous?** This scores a preloaded game piece but doesn't adapt to field conditions.

```java
// Static preload scoring
public Command getPreloadAuto() {
    return new SequentialCommandGroup(
        new ScorePreload(superstructure),
        new DriveForward(driveSubsystem, 3.0)
    );
}
```

However, static autonomous routines can't adapt to alliance partners, field conditions, or strategic needs that change throughout a competition.

**Strategic Autonomous** Autonomous routines designed around game analysis, alliance coordination, and robust execution provide:

- **Maximum point potential:** Score multiple game pieces efficiently
- **Strategic positioning:** End in advantageous locations for teleop
- **Alliance coordination:** Complement rather than conflict with partners  
- **Tiebreaker advantage:** Auto points often determine close matches
- **Consistent execution:** Work reliably across different field conditions

**Real Benefits**
- Significant competitive advantage in qualification matches
- Higher alliance selection value during top alliance scouting meetings
- Better playoff performance through coordination

---

## 2. Strategic Foundation

### Game Analysis - "What should we accomplish?"
Before writing any code, analyze what autonomous can achieve and work with the strategy subunit of your team to go over the following points:

**Key Questions:**
- What bonus points are available only during autonomous?
- How do autonomous points compare to teleop scoring rates?
- What field positioning advantages exist?
- How does autonomous performance affect tiebreakers?

**Example Analysis (2023 Charged Up):**
```
Mobility: 3 points (auto only)
Game pieces: 3-6 points depending on level  
Balance: 8-12 points depending on robot count
Tiebreaker: Auto points are 4th criteria (critical!)

Theoretical maximum: ~27 points in 15 seconds
Realistic target: 15-20 points consistently
```

### Time and Motion Study
```java
// Example timing breakdown for 2023
// Preload score: 2.0 seconds
// Drive to game piece: 3.5 seconds  
// Intake game piece: 1.5 seconds
// Return and score: 4.0 seconds
// Drive to balance: 3.0 seconds
// Balance: 1.0 seconds
// Total: 15.0 seconds (fully utilized)
```

### Alliance Considerations - "Who are we playing with?"

Autonomous routines play different roles during Qualification and Playoff matches. Here are some considerations to make about both types of matches when writing your routines.

**Qualification Matches:**
- Always assume most alliance partners will provide minimal coordination during alliance meetings
- Design routines that maximize your individual contribution
- Avoid areas where robot conflicts are likely

**Playoff Matches:**  
- Playing with skilled, coordinated alliance partners
- Design complementary routines (who does what?)
- Enable multiple strategic options

---

## Strategic Planning Template Planning Guide

### Phase 1: Game Analysis
1. **Rule Study** - Analyze current game manual for autonomous opportunities
2. **Point Calculation** - Determine theoretical maximum autonomous scores
3. **Timing Analysis** - Break down mechanism and movement times
4. **Competition Research** - Study what top teams accomplished in autonomous

### Phase 2: Strategic Planning
1. **Option Development** - Create 3-5 different autonomous routine concepts
2. **Alliance Scenarios** - Plan for different partnership situations
3. **Risk Assessment** - Evaluate reliability vs. scoring potential
4. **Prioritization** - Rank routines by strategic value

**Success Criteria:**
- Clear understanding of autonomous scoring opportunities
- Documented timing analysis for all major actions
- Multiple autonomous options for different scenarios
- Strategic reasoning for each autonomous choice

---

## Where to Go Next

**Ready for implementation? Continue with:**
- **[FRC Autonomous Programming - Implementation](./writing_autos.md)** – Technical implementation of your strategic plans

**Additional Strategic Resources:**
- [Game Analysis Frameworks](https://www.chiefdelphi.com/) - Community discussions on strategy
- [Competition Scouting](https://www.thebluealliance.com/) - Data-driven strategic decisions

---

**🎯 Ready to plan strategically?** Work through the strategic analysis systematically, then move on to technical implementation to bring your autonomous vision to life!

**Remember:** "A good auto is more than than fancy code." Strategic thinking and planning are the foundation of successful autonomous routines.

---