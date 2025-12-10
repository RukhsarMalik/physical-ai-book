--- 
id: cognitive-planning
title: LLMs for Robot Cognitive Planning
sidebar_position: 3
---

## 1. Introduction to Cognitive Planning with LLMs

**Cognitive planning** in robotics refers to the ability of a robot to reason about its goals, environment, and capabilities to generate a sequence of actions that achieves a desired outcome. Traditionally, this has involved complex symbolic AI, state-space search, or hand-coded finite state machines. The emergence of **Large Language Models (LLMs)** has opened up a new paradigm for cognitive planning, enabling robots to leverage the vast world knowledge and reasoning capabilities embedded within these models.

LLMs can bridge the gap between high-level, abstract human language instructions and the low-level, concrete actions a robot can execute. They act as a "brain" that can understand intent, contextualize commands, and propose executable plans.

## 2. Natural Language to Action Sequences

One of the most powerful applications of LLMs in robotics is their ability to translate vague or complex natural language instructions into a sequence of more specific, robot-executable actions.

**The process typically involves:**

1.  **Human Instruction**: A user provides a high-level command (e.g., "Go clean the table").
2.  **LLM Interpretation**: The LLM processes this command, leveraging its knowledge of the world to understand the intent (`clean`) and relevant objects/locations (`table`).
3.  **Action Decomposition**: The LLM decomposes the high-level command into a series of smaller, more manageable sub-goals or primitive actions that the robot can perform. This might involve generating a plan in natural language first, then translating those natural language steps into robot-specific commands.
4.  **Robot-Executable Format**: The decomposed actions are translated into a format directly usable by the robot's control system, often ROS 2 actions, services, or topics.

**Example: "Clean the room" → ROS 2 Action Plan**

Let's consider a complex instruction like "Clean the room." An LLM, given appropriate context and prompts, could generate a plan similar to this:

**Initial Human Command**: "Robot, please clean the living room."

**LLM-Generated High-Level Plan (Natural Language)**:
1.  Navigate to the living room.
2.  Identify any trash or clutter on the floor or table.
3.  Pick up each piece of trash.
4.  Place trash in the waste bin.
5.  Sweep the floor.
6.  Return to the charging station.

**Translation to ROS 2 Action Plan (Conceptual)**:
This natural language plan would then be translated into a sequence of ROS 2 actions or service calls. Each high-level step corresponds to one or more robot capabilities:

1.  `ROS2 Action: navigate_to_pose (target: living_room_coords)`
2.  `ROS2 Service: perceive_scene (area_of_interest: living_room)` -> returns `list_of_objects_to_pick`
3.  `LOOP through list_of_objects_to_pick:`
    *   `ROS2 Action: pick_object (object_id: current_object)`
    *   `ROS2 Action: place_object (target_location: waste_bin_coords)`
4.  `ROS2 Action: sweep_area (area: living_room_floor)`
5.  `ROS2 Action: navigate_to_pose (target: charging_station_coords)`

This detailed breakdown shows how an LLM can convert a single human command into a robust, executable plan for a robot.

## 3. LLM Prompt Engineering for Robotics

The effectiveness of an LLM in generating robot plans heavily relies on **prompt engineering**. This involves carefully crafting the input given to the LLM to elicit the desired output.

**Key considerations for prompt engineering in robotics:**

*   **Role-Playing**: Assigning the LLM a specific role (e.g., "You are a robotic task planner...").
*   **Context Provision**: Providing the LLM with information about the robot's capabilities, its environment, available tools, and current state.
*   **Output Format**: Specifying the desired format for the plan (e.g., bullet points, JSON, or a list of ROS 2 action calls).
*   **Few-Shot Examples**: Giving the LLM a few examples of input commands and their corresponding robot plans to guide its understanding.
*   **Constraints and Safety**: Explicitly instructing the LLM about safety protocols, forbidden actions, or physical limitations of the robot.

## 4. Code Example: Basic LLM Integration with Robotics

This conceptual Python code snippet demonstrates how you might integrate a GPT model to generate a high-level robot plan from a natural language command.

```python
import openai
import os

# --- Configuration ---
# Ensure OPENAI_API_KEY is set as an environment variable
openai.api_key = os.getenv("OPENAI_API_KEY")

# --- Function to get robot plan from LLM ---
def generate_robot_plan_with_llm(user_command: str) -> str:
    """
    Generates a high-level robot action plan from a user command using an LLM.
    """
    if not openai.api_key:
        return "Error: OpenAI API key not set."

    system_prompt = """You are a robotic task planner. Your goal is to convert high-level natural language instructions into a sequence of discrete, high-level robot actions. Focus on logical steps. Each step should be a clear, simple robot action. Do not generate code directly, just the sequence of natural language actions.

Available high-level robot capabilities:
- NAVIGATE_TO(location: str)
- PICK_UP(object: str)
- PLACE_AT(location: str)
- SWEEP(area: str)
- IDENTIFY(object: str, area: str)

Example:
User command: \"Go to the kitchen and grab a cup.\"
Plan:
1. NAVIGATE_TO(kitchen)
2. IDENTIFY(cup, kitchen)
3. PICK_UP(cup)
4. NAVIGATE_TO(start_location) # assuming robot returns to where it was commanded
"""

    user_prompt = f"User command: \"{user_command}\"\nPlan:"

    try:
        response = openai.chat.completions.create(
            model="gpt-4",  # Use a capable LLM
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.2, # Lower temperature for more deterministic planning
            max_tokens=200
        )
        plan_text = response.choices[0].message.content
        return plan_text
    except openai.APIConnectionError as e:
        return f"OpenAI API Connection Error: {e}. Check internet and API key."
    except openai.APIStatusError as e:
        return f"OpenAI API Status Error: {e.status_code} - {e.response.json()}. Check API key/model."
    except Exception as e:
        return f"An unexpected error occurred: {e}"

# --- Main execution example ---
if __name__ == "__main__":
    if not os.getenv("OPENAI_API_KEY"):
        print("Error: OPENAI_API_KEY environment variable not set.")
        print("Please set it before running: export OPENAI_API_KEY='YOUR_KEY'")
    else:
        command = "Clean the living room table and then come back."
        print(f"User command: {command}")
        plan = generate_robot_plan_with_llm(command)
        print("\n--- LLM-Generated Plan ---")
        print(plan)

        command_2 = "Find my phone on the sofa."
        print(f"\nUser command: {command_2}")
        plan_2 = generate_robot_plan_with_llm(command_2)
        print("\n--- LLM-Generated Plan ---")
        print(plan_2)
```
This example shows how an LLM can be prompted to generate a structured plan. The next step would be to parse this plan and map it to actual ROS 2 action goals or service calls, enabling the robot to execute the high-level instruction.
