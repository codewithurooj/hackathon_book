# Textbook Curriculum Validator

**Command**: `/textbook.validate`
**Type**: Custom Subagent
**Purpose**: Validate textbook content against hackathon curriculum requirements

## Description

This command spawns a specialized validation subagent that cross-checks written textbook content against the official hackathon curriculum PDF to ensure all required topics are covered.

## Usage

```bash
/textbook.validate [chapter_number]
```

**Examples:**
```bash
/textbook.validate 4        # Validate Chapter 4 against curriculum
/textbook.validate all      # Validate entire textbook
```

## What It Does

The validator subagent will:

1. **Read Curriculum Requirements**
   - Extract required topics from `Hackathon_ Physical AI & Humanoid Robotics Textbook.pdf`
   - Identify module-specific learning objectives
   - Parse week-by-week curriculum structure

2. **Scan Chapter Content**
   - Read all markdown files in specified chapter
   - Extract covered topics and concepts
   - Analyze depth of coverage

3. **Generate Coverage Report**
   ```markdown
   ## Chapter 4 Validation Report

   ### Required Topics (from PDF)
   - ✅ NVIDIA Isaac Sim introduction
   - ✅ Isaac ROS packages
   - ✅ cuVSLAM (Visual SLAM)
   - ✅ Nav2 for humanoid navigation
   - ⚠️  Synthetic data generation (partial - needs exercises)
   - ❌ Performance benchmarking (missing)

   ### Coverage Score: 85/100

   ### Recommendations:
   - Add Section 7: Synthetic Data Generation with hands-on exercise
   - Add performance benchmarking tutorial comparing CPU vs GPU
   ```

4. **Actionable Recommendations**
   - Identify missing sections
   - Suggest content additions
   - Flag incomplete coverage

## Validation Criteria

### Chapter-Level Checks
- ✅ All curriculum topics mentioned in PDF
- ✅ Learning objectives aligned with course goals
- ✅ Appropriate depth for target audience
- ✅ Prerequisites clearly stated

### Content Quality Checks
- ✅ Code examples tested and working
- ✅ Technical accuracy
- ✅ Appropriate diagrams referenced
- ✅ Exercises align with learning objectives

### Hackathon-Specific Checks
- ✅ Word count reasonable (not excessive for hackathon scope)
- ✅ Docusaurus-compatible markdown
- ✅ Links to other chapters valid
- ✅ Images/assets referenced correctly

## Output Format

```yaml
chapter: 4
curriculum_match: 85%
missing_topics:
  - "Performance profiling with NVIDIA Nsight"
  - "Sim-to-real transfer validation"
partially_covered:
  - topic: "Synthetic data generation"
    current: "Conceptual explanation only"
    needed: "Hands-on exercise with domain randomization"
recommendations:
  - priority: high
    action: "Add Section 9: Performance Optimization"
    reason: "Required by curriculum Week 10"
```

## Integration with Workflow

Use after content creation:

```bash
# 1. Write content
/sp.implement

# 2. Validate against curriculum
/textbook.validate 4

# 3. Address gaps
# (create missing sections based on recommendations)

# 4. Re-validate
/textbook.validate 4
```

## Success Criteria

- ✅ All required curriculum topics covered (100%)
- ✅ No missing sections from PDF requirements
- ✅ Learning objectives align with course goals
- ✅ Content depth appropriate for timeframe (10-12 hours per module)

## Notes

- Validation is **non-destructive** - only reports findings
- Can run multiple times during development
- Useful for iterative improvement before submission
