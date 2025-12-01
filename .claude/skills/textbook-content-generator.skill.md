# Textbook Content Generator Skill

**Skill Type**: Content Generation
**Context**: Physical AI & Humanoid Robotics Textbook
**Reusable**: Yes - Can be used for any technical educational content

## Purpose

Generate high-quality technical textbook content following educational best practices for Physical AI and Robotics education.

## When to Use

- Writing new chapter sections
- Expanding existing content
- Creating exercises and assessments
- Generating code examples with explanations

## Input Requirements

```yaml
topic: "Chapter topic (e.g., ROS 2 Publishers, Isaac ROS VSLAM)"
section_type: "intro | tutorial | concept | exercise | assessment"
target_audience: "undergraduate engineering students"
word_count: "1500-2500 words"
learning_objectives: ["objective 1", "objective 2", ...]
prerequisites: ["prerequisite knowledge 1", "prerequisite 2", ...]
```

## Output Format

```markdown
# Section Title

## Introduction
[2-3 paragraphs introducing the topic]

## Core Concepts
[Detailed explanations with examples]

## Hands-On Tutorial (if applicable)
[Step-by-step implementation]

## Review Questions
[4-5 questions with expandable answers]

## Next Steps
[Link to next section]
```

## Quality Standards

1. **Educational Clarity**
   - Define technical terms on first use
   - Use analogies for complex concepts
   - Progress from simple to complex

2. **Code Examples**
   - Always include complete, runnable code
   - Add inline comments explaining key lines
   - Show expected output

3. **Visual Aids**
   - Reference diagrams where helpful
   - Use code blocks with syntax highlighting
   - Include command-line examples

4. **Assessment**
   - Include review questions
   - Provide detailed answer explanations
   - Link to related concepts

## Example Invocation

```
Use the textbook-content-generator skill to write a 2000-word tutorial on:
- Topic: Isaac ROS cuVSLAM
- Section type: tutorial
- Learning objectives: ["Install cuVSLAM", "Run VSLAM with Isaac Sim", "Build and save maps"]
```

## Success Criteria

- ✅ Meets word count target (±10%)
- ✅ All learning objectives addressed
- ✅ Code examples tested and working
- ✅ Review questions align with content
- ✅ Appropriate for target audience level
