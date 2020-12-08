hwAvg         = 99.3
collabAvg     = 798*100/800
quizAvg       = 87.3
midtermAvg    = 94
participation = 80 # the approximate median participation grade

# 2. Then pick these values based on your expected grades:
final = midtermAvg         # the default if you do not opt-in to take the final
tp = 95                    # your tp3 grade.  You can try different values...
passedParticipation = (participation >= 60)

# 3. Then run this to see your standard (non-AMG) grade computation:
grade1 = (0.10*quizAvg +
          0.15*hwAvg +
          0.20*collabAvg +
          0.15*midtermAvg +
          0.10*participation +
          0.20*tp +
          0.10*final)
divisor = 0.9 if passedParticipation else 1.0
grade2 = (0.10*quizAvg +
          0.15*hwAvg +
          0.20*collabAvg +
          0.15*midtermAvg +
          0.20*tp +
          0.10*final) / divisor
grade = max(grade1, grade2)

if (grade2 > grade1):
    print('Using pass-fail participation grade as that is worth more.')
print(f'Standard (non-AMG) grade: {round(grade, 1)}')

if (grade < 70):
    print('Please recompute using the AMG grade computation.')
    print('See syllabus for details.')