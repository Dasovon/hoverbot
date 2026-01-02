# Starting New AI Chat Session

## Step 1: Upload Repository

1. Create fresh zip of repository:
```bash
   cd ~
   zip -r hoverbot-snapshot-$(date +%Y%m%d).zip hoverbot \
     -x "hoverbot/ros2_ws/build/*" \
     -x "hoverbot/ros2_ws/install/*" \
     -x "hoverbot/ros2_ws/log/*" \
     -x "hoverbot/.git/*"
```

2. Upload to new chat

## Step 2: First Message
```
Hi! I'm continuing development on the HoverBot autonomous robot project.

I've uploaded the complete repository. Please read:
1. docs/DEVELOPMENT_CONTEXT.md (complete session context)
2. docs/CLAUDE_CODE_WORKFLOW.md (development workflow)
3. docs/PROJECT_ROADMAP.md (next steps)

Setup:
- Dev machine: Ubuntu desktop with VSCode
- Robot: Raspberry Pi 4 at ssh hoverbot (Ubuntu 22.04 + ROS 2 Humble)
- Workflow: Dev pushes to GitHub → Pi pulls
- Tools: Claude Code (code generation) + you (planning/architecture)

Current focus: [describe what you want to work on]

Can you confirm you've read the context and understand the project state?
```

## Step 3: Verify Context Loaded

Ask the AI:
- "What's the current project status?"
- "What are the critical constraints for firmware?"
- "What's our development workflow?"

If answers match context file → proceed
If answers don't match → re-share context file

## Step 4: Start Development

Follow workflow in docs/CLAUDE_CODE_WORKFLOW.md
Reference docs/DEVELOPMENT_CONTEXT.md for constraints
Update context file when making significant changes