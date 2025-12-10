---
id: 15
title: Implement Task T012 - Create exercise.md
stage: green
date: 2025-11-29
surface: "agent"
model: "gemini"
feature: "001-ros2-fundamentals"
branch: "001-ros2-fundamentals"
user: "user"
command: "/sp.implement"
labels: ["implement", "docusaurus", "content"]
links:
  spec: "specs/001-ros2-fundamentals/spec.md"
  plan: "specs/001-ros2-fundamentals/plan.md"
  tasks: "specs/001-ros2-fundamentals/tasks.md"
  adr: "null"
  pr: "null"
files:
 - specs/001-ros2-fundamentals/tasks.md
 - physical-ai-book/docs/module-1-ros2-fundamentals/exercise.md
tests:
 - "none"
---

## Prompt

/sp.implement Execute Task T012: Create exercise.md with hands-on tutorial

Create the file docs/module-1-ros2-fundamentals/exercise.md with a complete step-by-step tutorial:

1. Exercise Overview
   - What you'll build
   - Prerequisites
   - Estimated time: 30-45 minutes

2. Step 1: Create ROS 2 Workspace
   - Commands to create workspace
   - Folder structure explanation

3. Step 2: Create a Package
   - Using ros2 pkg create command
   - Package structure

4. Step 3: Write Publisher Node
   - Create Python file
   - Complete code for talker node
   - Explanation

5. Step 4: Write Subscriber Node
   - Create Python file
   - Complete code for listener node
   - Explanation

6. Step 5: Build the Package
   - colcon build commands
   - Source the workspace

7. Step 6: Run the Nodes
   - Terminal commands
   - Expected output
   - Verification steps

8. Step 7: Experiment and Modify
   - Suggestions for modifications
   - Challenge exercises

9. Troubleshooting
   - Common errors and solutions

Include:
- All terminal commands
- Complete code examples
- Expected outputs
- Verification checkpoints

Write 1200-1500 words with complete tutorial.

Create this file now.

## Response snapshot

Successfully executed Task T012 and created the `exercise.md` file with a comprehensive hands-on tutorial. The task has been marked as complete in `specs/001-ros2-fundamentals/tasks.md`.

## Outcome

- ✅ Impact: High - The practical exercise page for Module 1 has been created, providing a hands-on learning experience. All content creation tasks for Module 1 are now complete.
- 🧪 Tests: N/A
- 📁 Files: 2 files modified (`tasks.md`, `physical-ai-book/docs/module-1-ros2-fundamentals/exercise.md`).
- 🔁 Next prompts: Continue with the next task, `/sp.implement Execute Task T013`.
- 🧠 Reflection: The task executed successfully. This completes all content generation for Module 1.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): None
- Next experiment (smallest change to try): N/A
