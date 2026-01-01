# Claude Code + claude.ai Workflow Guide

**Last Updated:** 2026-01-01  
**Based On:** [Anthropic's Best Practices for Agentic Coding](https://www.anthropic.com/engineering/claude-code-best-practices)

---

## 🎯 Core Principles

### Trunk-Based Development
- **main** branch is always stable and deployable
- Feature branches are **small, short-lived** (1-3 days max)
- **One branch = one bounded task**
- **Merge fast, merge often**

### Small, Atomic Changes
- Each commit does **one logical thing**
- PRs are **reviewable in minutes**, not hours
- Avoid "misc cleanup" or mixed concerns
- **Small scope = high reliability**

### Explicit Instructions
- Claude Code is a **pair programmer**, not an autonomous agent
- **Be specific** about intent, constraints, and what NOT to change
- **Repeat context** when needed - Claude doesn't retain state between sessions
- Clear success criteria up front

---

## 🛠️ Tool Roles

### This Chat (claude.ai) - The Architect
**Purpose:** Strategic thinking, context management, architectural decisions

**Use For:**
- Discussing features and approaches
- Making architectural decisions
- Drafting explicit instructions for Claude Code
- Reviewing Claude Code's output
- Maintaining project context
- Troubleshooting complex issues
- Planning multi-step workflows

**NOT For:**
- Writing code directly (use Claude Code for that)
- File creation (Claude Code does this)

---

### Claude Code (Web + VSCode) - The Pair Programmer
**Purpose:** Execute specific, bounded coding tasks

**Use For:**
- Creating new files from clear specifications
- Editing existing files with explicit instructions
- Running tests and verifying changes
- Small, atomic code modifications
- Following step-by-step instructions

**Best Practices:**
- Give it **existing structure** whenever possible (Claude is better at editing than inventing)
- One clearly scoped task per session
- Explicit constraints: "DO NOT modify X", "PRESERVE Y"
- Clear success criteria
- Reference the context document

**NOT For:**
- Vague requests like "improve this"
- Architectural redesigns without guidance
- Multi-file refactors without clear scope
- Speculative changes

---

### You (Developer) - The Driver
**Purpose:** Review, test, decide, ship

**Responsibilities:**
- Review ALL changes from Claude Code
- Test locally before merging
- Make final decisions
- Commit and push
- Manage branches
- Update context documentation
- Maintain project quality

---

### GitHub CLI - Version Control
**Purpose:** Fast, password-free git operations

**Use For:**
```bash
# Quick commits
git add .
git commit -m "feat: Add feature"
git push

# Create PRs
gh pr create --base main --head feature/name

# Merge PRs
gh pr merge --squash

# Check status
gh pr list
gh repo view
```

---

## 🌿 Branch Strategy

### Structure
```
main (always stable, production-ready)
  └── Short-lived feature branches:
      ├── feature/gazebo-tuning
      ├── feature/nav2-config
      ├── feature/power-system
      └── bugfix/lidar-timeout
```

### Branch Lifecycle
```bash
# 1. Create from main
git checkout main
git pull origin main
git checkout -b feature/descriptive-name

# 2. Work (with Claude Code)
# ... make changes ...

# 3. Push and PR
git push origin feature/descriptive-name
gh pr create --base main --head feature/descriptive-name

# 4. Merge
gh pr merge --squash

# 5. Cleanup
git checkout main
git pull origin main
git branch -d feature/descriptive-name
git push origin --delete feature/descriptive-name
```

### Naming Convention
- `feature/short-description` - New functionality
- `bugfix/issue-description` - Bug fixes
- `docs/what-changed` - Documentation only
- `refactor/component-name` - Code improvements

**Examples:**
- ✅ `feature/nav2-params`
- ✅ `bugfix/lidar-timeout`
- ✅ `docs/gazebo-setup`
- ❌ `improvements` (too vague)
- ❌ `working-branch` (not descriptive)

---

## 📝 Context Management

### CLAUDE_CONTEXT.md
**Location:** `docs/CLAUDE_CONTEXT.md`

**Purpose:** Single source of truth for Claude Code sessions

**Update Before Each Claude Code Session**

**Template:**
```markdown
# Claude Code Context Document

**Last Updated:** YYYY-MM-DD

## Project Overview
[One-paragraph project description]

## Current State
- **Software:** [status]
- **Hardware:** [status]
- **Testing:** [status]

## Active Constraints
1. **DO NOT** modify [file/component] - [reason]
2. **PRESERVE** [feature] - [why it matters]
3. **MAINTAIN** [compatibility/standard]

## Architecture Decisions
- [Key decision 1 and rationale]
- [Key decision 2 and rationale]

## File Ownership
- `path/to/critical/file/` - [what it does, why not to auto-refactor]

## Current Focus
[What you're working on right now]

## Common Pitfalls to Avoid
- [Mistake 1 and consequence]
- [Mistake 2 and consequence]
```

---

## 🔄 The Workflow Loop

### Complete Feature Development Cycle

**Estimated Time Per Feature:** 20-30 minutes

---

#### 1️⃣ Planning (This Chat - 5 min)

**In claude.ai chat:**
```
You: "I want to add [feature description]"

Architect (claude.ai):
- Discusses approach
- Identifies constraints
- Suggests architecture
- Drafts explicit instructions for Claude Code
- Identifies files to modify
```

**Output:** Clear, explicit instructions ready to paste into Claude Code

---

#### 2️⃣ Create Branch (Terminal - 1 min)
```bash
cd ~/hoverbot
git checkout main
git pull origin main
git checkout -b feature/descriptive-name
```

---

#### 3️⃣ Update Context (Dev - 2 min)

**Edit:** `docs/CLAUDE_CONTEXT.md`

**Add:**
- Current focus
- Any new constraints
- Files that will be modified
- Success criteria

**Commit:**
```bash
git add docs/CLAUDE_CONTEXT.md
git commit -m "docs: Update context for [feature]"
```

---

#### 4️⃣ Prepare Instructions (This Chat - 5 min)

**In claude.ai chat, architect provides:**

**Example Instruction Block:**
```
Tell Claude Code:

"Create nav2_params.yaml in ros2_ws/src/hoverbot_navigation/config/

Specifications:
- Controller frequency: 20 Hz
- Planner: DWB controller
- Recovery behaviors: standard set

Constraints:
- DO NOT modify existing launch files
- DO NOT change hoverbot_driver code
- PRESERVE existing SLAM configuration
- MAINTAIN ROS 2 Humble compatibility

Expected files:
- config/nav2_params.yaml (new)
- README.md (update usage section)

Refer to docs/CLAUDE_CONTEXT.md for project constraints."
```

---

#### 5️⃣ Execute (Claude Code - 5-10 min)

**In Claude Code (web or VSCode):**

1. **Paste exact instructions** from claude.ai chat
2. **Reference context:** "Read docs/CLAUDE_CONTEXT.md first"
3. **Review changes** before accepting
4. **Let Claude Code commit**

**Claude Code will:**
- Read context document
- Create/modify files
- Commit with clear message
- Push to branch (if configured)

---

#### 6️⃣ Review & Test (Local - 5 min)

**Dev - Terminal:**
```bash
# Check what changed
git diff main

# Build and test
cd ~/hoverbot/ros2_ws
colcon build --packages-select [package-name]
source install/setup.bash

# Test the feature
ros2 launch [test-command]

# Verify sensor topics still work
ros2 topic list
ros2 topic hz /scan
```

**If issues found:**
- Go back to claude.ai chat
- Discuss problem
- Get refined instructions
- Return to Claude Code

---

#### 7️⃣ Merge (Terminal - 3 min)

**If tests pass:**
```bash
# Push branch
git push origin feature/descriptive-name

# Create PR
gh pr create \
  --base main \
  --head feature/descriptive-name \
  --title "feat: Add [feature]" \
  --body "Description of changes

- Created: [files]
- Modified: [files]
- Tested: [how]"

# Review PR in GitHub
# If good, merge
gh pr merge --squash

# Cleanup
git checkout main
git pull origin main
git branch -d feature/descriptive-name
git push origin --delete feature/descriptive-name
```

---

#### 8️⃣ Document (This Chat - 2 min)

**Back to claude.ai chat:**
```
You: "[Feature] complete, merged to main"

Architect:
- Updates mental context
- Suggests next steps
- Identifies related work
```

---

## 📋 Instruction Templates for Claude Code

### Creating New Files
```
Create [filename] in [path]

Purpose: [what this file does]

Specifications:
- [specific requirement 1]
- [specific requirement 2]

Constraints:
- DO NOT modify [existing files]
- PRESERVE [existing functionality]
- MAINTAIN [standard/compatibility]

Expected output:
- [new file 1]
- [new file 2] (if needed)

Reference docs/CLAUDE_CONTEXT.md for project constraints.
```

---

### Modifying Existing Files
```
Edit [filename] to [specific change]

Context: [why this change is needed]

Changes needed:
1. [specific edit 1]
2. [specific edit 2]

DO NOT:
- Change [unrelated functionality]
- Modify [critical section]
- Remove [important feature]

PRESERVE:
- [existing behavior]
- [API compatibility]

Test by: [how to verify it works]
```

---

### Multi-Step Tasks
```
Task: [overall goal]

Step 1: [specific first step]
- Create/modify: [files]
- Requirements: [specs]

Step 2: [specific second step]
- Create/modify: [files]
- Requirements: [specs]

After each step:
- Verify [specific outcome]
- Show diff before proceeding

Constraints for entire task:
- DO NOT [constraint 1]
- PRESERVE [constraint 2]

Refer to docs/CLAUDE_CONTEXT.md before starting.
```

---

## 🚫 Anti-Patterns (Don't Do This)

### ❌ Vague Instructions
**Bad:** "Improve the launch files"  
**Good:** "Add use_sim_time parameter to hoverbot_full_v3.launch.py, default to false"

---

### ❌ Scope Creep
**Bad:** "Add Nav2 and also refactor the driver and update docs"  
**Good:** "Add nav2_params.yaml with DWB controller config"

---

### ❌ Assuming Context
**Bad:** "Fix the bug we discussed"  
**Good:** "Fix LiDAR timeout by increasing watchdog to 45s in lidar_manager.py line 87"

---

### ❌ Long-Lived Branches
**Bad:** Keep feature branch open for 2 weeks  
**Good:** Merge within 1-3 days, create new branch for next iteration

---

### ❌ Mixed Concerns in One Commit
**Bad:** "Update launch files, add docs, fix bug, refactor driver"  
**Good:** Four separate commits/branches

---

## 🎯 Common Workflows

### Adding a New ROS 2 Package

**1. Plan (claude.ai):**
```
You: "Need new package for navigation"
Architect: [provides structure, dependencies, package.xml template]
```

**2. Instruct Claude Code:**
```
Create new ROS 2 package: hoverbot_navigation

Location: ros2_ws/src/hoverbot_navigation

Structure:
- package.xml (ament_cmake, dependencies: nav2_common, nav2_bt_navigator)
- CMakeLists.txt (minimal, install config/ and launch/)
- config/ directory
- launch/ directory
- README.md

DO NOT modify existing packages.

Refer to docs/CLAUDE_CONTEXT.md.
```

**3. Test:**
```bash
colcon build --packages-select hoverbot_navigation
```

---

### Updating Configuration Files

**1. Plan (claude.ai):**
```
You: "Need to increase EKF frequency from 50Hz to 100Hz"
Architect: [identifies file, discusses tradeoffs, provides exact change]
```

**2. Instruct Claude Code:**
```
Edit ros2_ws/src/hoverbot_bringup/config/ekf.yaml

Change:
- Line 35: frequency: 50.0 → frequency: 100.0

DO NOT modify any other parameters.
PRESERVE all sensor configurations.

Commit message: "config: Increase EKF frequency to 100Hz"
```

---

### Debugging Issues

**1. Discuss (claude.ai):**
```
You: "LiDAR stops after 30s, getting timeout errors"
Architect: [analyzes logs, identifies issue, suggests fix]
```

**2. Instruct Claude Code:**
```
Edit ros2_ws/src/hoverbot_bringup/hoverbot_bringup/lidar_manager.py

Line 87: Change watchdog timeout from 30.0 to 45.0

Reason: Current timeout too aggressive for simulation testing.

DO NOT modify power management logic.
PRESERVE bench_test_mode functionality.

Test by:
- Launch full system
- Wait 40 seconds without movement
- Verify LiDAR stays active
```

---

## 📊 Quality Checklist

**Before Merging ANY PR:**

- [ ] Changes match original intent
- [ ] No unrelated modifications
- [ ] Tests pass locally
- [ ] Documentation updated (if needed)
- [ ] Commit message is clear
- [ ] No hardcoded values (unless intentional)
- [ ] Error handling preserved
- [ ] ROS 2 Humble compatibility maintained
- [ ] Sensor configurations unchanged (unless that's the goal)
- [ ] Branch is up to date with main

---

## 🔧 Troubleshooting

### Claude Code Made Unwanted Changes

**Solution:**
```bash
# Don't accept the changes
# Return to claude.ai chat
# Refine instructions with more explicit constraints
# Try again
```

### Context Drift Between Sessions

**Solution:**
- Always update docs/CLAUDE_CONTEXT.md before Claude Code sessions
- Reference it explicitly in instructions: "Read docs/CLAUDE_CONTEXT.md first"
- Repeat critical constraints in every instruction block

### Merge Conflicts

**Solution:**
```bash
# Small, frequent merges prevent this
# If it happens:
git checkout main
git pull origin main
git checkout feature/branch-name
git merge main
# Resolve conflicts manually
# Continue
```

### Claude Code Doesn't Understand Project

**Solution:**
- Check docs/CLAUDE_CONTEXT.md is up to date
- Provide more specific file paths
- Reference existing similar code
- Break task into smaller steps

---

## 📈 Success Metrics

**You're doing it right when:**
- ✅ Features go from idea → merged in < 1 hour
- ✅ PRs are under 200 lines of changes
- ✅ Branches live less than 3 days
- ✅ Main branch is always deployable
- ✅ You rarely have merge conflicts
- ✅ Claude Code rarely guesses wrong
- ✅ Code reviews take minutes, not hours

**Warning signs:**
- ⚠️ Branches open for weeks
- ⚠️ PRs with 1000+ line changes
- ⚠️ Frequent "undo" or "revert" commits
- ⚠️ Claude Code making unexpected changes
- ⚠️ Merge conflicts on every PR
- ⚠️ Tests breaking after merges

---

## 🎓 Learning Resources

**Anthropic Documentation:**
- [Best Practices for Agentic Coding](https://www.anthropic.com/engineering/claude-code-best-practices)
- [Claude Code Documentation](https://code.claude.com/docs/en/overview)

**Trunk-Based Development:**
- https://trunkbaseddevelopment.com/

**ROS 2 Best Practices:**
- https://docs.ros.org/en/humble/

---

## 📝 Workflow Summary Card

**Print this out and keep it handy:**
```
┌─────────────────────────────────────────────┐
│  HOVERBOT DEVELOPMENT WORKFLOW              │
├─────────────────────────────────────────────┤
│                                             │
│  1. PLAN (claude.ai chat)                   │
│     → Discuss approach                      │
│     → Get explicit instructions             │
│                                             │
│  2. BRANCH (terminal)                       │
│     → git checkout -b feature/name          │
│                                             │
│  3. CONTEXT (update CLAUDE_CONTEXT.md)      │
│     → Add current focus                     │
│     → List constraints                      │
│                                             │
│  4. EXECUTE (Claude Code)                   │
│     → Paste instructions                    │
│     → Reference context doc                 │
│     → Review changes                        │
│                                             │
│  5. TEST (local)                            │
│     → colcon build                          │
│     → ros2 launch [test]                    │
│                                             │
│  6. MERGE (terminal)                        │
│     → gh pr create                          │
│     → gh pr merge --squash                  │
│     → git checkout main && git pull         │
│                                             │
│  7. DOCUMENT (claude.ai chat)               │
│     → Report results                        │
│     → Get next steps                        │
│                                             │
│  TIME: ~25 minutes per feature              │
│                                             │
└─────────────────────────────────────────────┘

KEY PRINCIPLES:
- Small tasks
- Explicit instructions  
- Fast merges
- Main always stable
```

---

## ✅ Ready to Start

**Your workflow is:**
1. **claude.ai** for planning & context
2. **Claude Code** for coding
3. **GitHub CLI** for merging
4. **Small branches** merged fast

**Next:** Follow the workflow loop for your first feature!

---

**Last Updated:** 2026-01-01  
**Maintained By:** Project team  
**Review Frequency:** Update before each major milestone