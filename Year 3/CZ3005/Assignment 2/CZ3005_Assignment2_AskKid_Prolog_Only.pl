/* Dynamic predicates*/
:-dynamic asked/1.
:-dynamic answered_yes/1.
:-dynamic answered_yes_additional/1.
:-dynamic answered_no/1.

/* NOTE: Assumptions made in this project:
 *  If the subject is not done in school i.e. kid answeres no, advice
 *  that follows assumes that subject happened but kid did not
 *  participate. */

/* Example of a flow for this program: ask(0) -> ask_subject(Y) -> ask_subject_list(Y) -> ask_subject_list_additional(Y) -> validate_follow_up(Y,X) -> ask_subject(Y) 
								-> process_advice(Y) -> ask_subject(Y) */
/* in plain text : ask a subject Y to kid -> ask all activities X in subject Y -> ask follow-up questions X regarding subject Y -> give a conclusion F depending on question X of subject Y -> find a new subject Y and ask subject Y to kid again*/

/* Start with first question -> Study
 *  Typically parent will be more concerned about study. */
ask(0):- ask_subject(study).

/* ask_subject : the general purpose of this is to lead subject Y into its respective branch depending on
*the child's reply. If the child replied yes, the program will dive in
*to the activities of subject Y and proceed to ask further (top-down
*approach). If answered no, a closure to the subject, some sort of
*advice, will be given
*and proceed to the next subject */
ask_subject(Y) :- print('Did you '), print(Y), print(' today? y/n/q: '), read(Ans), assert(asked(Y)),
      (Ans == q -> abort; Ans == y -> validate_and_process_subject(Y); assert(answered_no(Y)), process_advice(Y),  next_subject(Y)).

/* ask_subject_list : Upon receiving yes from a kid for a subject Y, the program will launch generate_options to retrieve a list of
*valid options (that has not been asked) and a random member within it to the child.
*IF the list is empty, that means all possible activies has been asked, random_member will return false and proceed to find the next subject to ask through next_subject. */
ask_subject_list(Y) :- generate_options(Y, Valid), random_member(X, Valid), process_qn(X,Y);next_subject(Y).

/* ask_subject_list_additional : Similarly to the above, if there is a follow_up list of subject Y after activities has been asked, ggenerate_options_additional will return a list and asked to the kid. These follow_up list of questions are typically questions and statements that shows concern to the kid based on the subject Y. For example, eat -> [] -> 'Did you wash hands?' -> conclusion (later explained).
*/
ask_subject_list_additional(Y) :- generate_options_additional(Y,Valid), random_member(X,Valid), process_qn2(X,Y).

/* to find and proceed to asking the next subject. this function is required to link to verify_if_no_subjectList for the term (Y,L) <- when list is empty, refers to that no activities have been done by the child therefore, will not follow up with anymore questions like 'Did you wash hands'. */
ask_answered_no_subject(Y,[]):- generate_options(subject,Valid), random_member(X,Valid), ask_subject(X).

/* redirects to ask additional message (follow up questions) of subject Y if term (Y,L) L is not empty ([]). */
ask_answered_no_subject(Y,Valid):- ask_subject_list_additional(Y).

/* Determining whether additional_list should be asked using intersection and answered_yes(X) predicate. The logic behind this is that, if no activities behind a subject has been done, then follow up questions would not be required to be asked.
* If Valid list is [] -> launch ask_answered_no_subject(Y,[])
* If Valid list is not [] -> launch ask_anwered_no_subject(Y,Valid)
*This achieves some sort of an method overloading effect, thus reducing recursions error */
verify_if_no_subjectList(Y):- related(Y,L), findnsols(100,I,answered_yes(I),AskedList_additional),intersection(L,AskedList_additional,Valid),ask_answered_no_subject(Y,Valid).

/* process_qn : to handle printing of subject Y and activity X (first level) and its respective assert-to List.
Once asserting of KB is done, relaunch ask_subject_list(Y) to find next activity to ask. */
process_qn(X,Y) :- print('Did you '), print(Y),print(' '), print(X), print(' today? y/n/q: '), read(Ans), assert(asked(X)),
				   (Ans == q->abort; Ans ==y -> assert(answered_yes(X)); Ans ==n->assert(answered_no(X))), ask_subject_list(Y).

/* process_qn2 : to handle printing of subject Y and follow_up questions X of Y e.g. Subject = eat, X will be an element in the list of follow_up_eat(L). Depending on the reply of the kid, the program will follow up with a 'conclusion' to end the concern statement - done through validate_follow_up(Y,X). If no validate_follow_up(Y,X) can be found i.e. no appropriate conclusion to the concern statement, the program will just proceed back to ask_subject_list_additional(Y). For example: X = 'Did you drink water?', there is a valid validate_follow_up(eat,'Did you drink water?') -> program will process the necessary assert and draw a conclusion. */
process_qn2(X,Y) :- print(Y), print(' - '), print(X), print('? y/n/q: '), read(Ans), assert(asked(X)),
					(Ans ==q->abort; Ans==y -> assert(answered_yes_additional(X)); Ans==n-> assert(answered_no(X))),
					validate_follow_up(Y,X),ask_subject_list_additional(Y); ask_subject_list_additional(Y).

/* process_advice : to print a random advice from follow_up_no(Y,L), this happens when the kid replies no to participating in a subject. Showcases a parent's concern and advice to particular subject Y. For example: If kid did not exercise, parent should reply an appropriate concern or advice to encourage the kid to participate next time! */
process_advice(Y) :- follow_up_no(Y,L), random_member(X,L), print(X),nl.

/* next_subject(Y) : There is a few parts to this function, Firstly, if answered_yes(Y) is true i.e. kid did the subject Y, a check will be done to verify if all activities has been answered no or yes.
Secondly, if answered_yes(Y) is false i.e. kid did not do subject (a follow up from above), the program will find a random_member from Valid list and asked to the kid.
Lastly, if random_member(X,Valid) is false, that means all subjects have
been asked and therefore, program will end.
Ends with congratulatory message if no more member in Valid */
next_subject(Y) :- answered_yes(Y),(verify_if_no_subjectList(Y));
				   generate_options(subject,Valid), random_member(X,Valid), ask_subject(X);
				   print('Good job for today! Go and rest').

/* asserts answered_yes and proceed with asking activities in subject */
validate_and_process_subject(Y) :- assert(answered_yes(Y)), ask_subject_list(Y).

/* Generation of Valid list through set differences between AskedList and Subject's list */
generate_options(Y,Valid) :-
related(Y,L), findnsols(100,I,asked(I),AskedList), subtract(L,AskedList,Valid).

/* Generation of Valid list through set differences between AskedList and Subject's follow_up list */
generate_options_additional(Y,Valid) :-
follow_up(Y,L),findnsols(100,I,asked(I),AskedList_additional),subtract(L,AskedList_additional,Valid).

/*Populates activities list L of subject Y or Y input */
related(subject,L) :- subject(L).
related(study,L) :- study(L).
related(play,L) :- play(L).
related(eat,L) :- eat(L).
related(see,L) :- see(L).
related(dance,L):- dance(L).
related(talk,L):- talk(L).
related(exercise,L):- exercise(L).

/*Populates follow_up questions list L based on subject Y
This list contains questions or statements that express concern to the kid's activities. The purpose of this is for more interaction and shows a more intimate relationship. */
follow_up(eat,L) :- follow_up_eat(L).
follow_up(play,L) :- follow_up_play(L).
follow_up(exercise,L) :- follow_up_exercise(L).
follow_up(study,L) :- follow_up_study(L).
follow_up(dance,L) :- follow_up_dance(L).
follow_up(talk,L) :- validate_follow_up(talk,strangers).
follow_up(see,L) :- follow_up_see(L).

/*Populates follow_up_no questions list L based on subject Y
This list contains advice or concern statements to show the conclusion of a subject when the kid replies no when asked a subject. */
follow_up_no(eat,L) :- follow_up_no_eat(L).
follow_up_no(play,L) :- follow_up_no_play(L).
follow_up_no(exercise,L) :- follow_up_no_exercise(L).
follow_up_no(study,L) :- follow_up_no_study(L).
follow_up_no(dance,L) :- follow_up_no_dance(L).
follow_up_no(talk,L) :- follow_up_no_talk(L).
follow_up_no(see, L) :- follow_up_no_see(L).

/*Predicates, Facts*/
/*Common activities or objects that can be found or done in school*/
subject([play,eat,see,study,exercise,dance,talk]).
play([swing,slides,hide_and_seek,catching,in_the_mud,sandbox,soft_toys,playmat,toys]).
eat([vegetable,fries,ice-cream, meat, croquettes, bread, noodles, spaghetti, seafood, japanese_food]).
talk([friends,strangers,teachers]).
see([planes,big_trucks,train,strange_clouds, birds,cats,dogs]).
study([alphabets, mathematics, science, language, plants, trees, animals]).
exercise([jumping_jacks, jogging, sit-ups, pull-ups, skip_rope, squats]).
dance([cartoon_songs,kpop_songs,classical_music]).

/*Questions or Statements that shows further concern to the child's activities of subject Y, provides higher interactivity.
Common concerns that a parent can have for a child to further deepen the relationship. */
follow_up_play(['Did you injure yourself?','Did you enjoy yourself?']).
follow_up_eat(['Did you wash your hands?','Was it tasty?','Did you drink water?']).
follow_up_study(['Was if fun?','Was it easy?','Do you need help with homework?']).
follow_up_exercise(['Did you have fun?','Did you fall down?']).
follow_up_dance(['Did you bump into anybody?','Are you able to remember the steps?']).
follow_up_talk([]).
follow_up_see(['Did you see anything you do not know about?','Was there any interesting objects?']).

/* Some advice, if exist, if 'no' was answered to acitivty in school. */
follow_up_no_play(['You should go out and play with your friends.']).
follow_up_no_eat(['You must be hungry! Let me cooked your favourite dish later.']).
follow_up_no_study(['It is normal to have no feels to study, just rest and recharge before continuing the journey!']).
follow_up_no_exercise(['Exercising is good for health!','Exercising will strengthen your body!']).
follow_up_no_dance(['Dancing is another form of exercise, you should try next time!']).
follow_up_no_talk(['Talking to people helps to build your confidence!']).
follow_up_no_see(['Seeing more things helps to broaden your knowledge!']).

/* For encouragement or conclusion messages to a particular question by follow_up_<Y>. This gives some sort of 'closure' to a question asked to the kid. */
/* For any subject Y and follow up question X, if there is a validate_follow_up(Y,X) that follows, then a specific function follow_up_<Y>_final(X,F) will be run, and print of variable F message thereafter. */
/* PLAY */
validate_follow_up(play,'Did you injure yourself?'):- follow_up_play_final('Did you injure yourself?',F),print(F),nl.
validate_follow_up(play,'Did you enjoy yourself?'):- follow_up_play_final('Did you enjoy yourself?',F),print(F),nl.
/* EAT */
validate_follow_up(eat,'Did you wash your hands?'):- follow_up_eat_final('Did you wash your hands?', F), print(F),nl.
validate_follow_up(eat,'Did you drink water?'):- follow_up_eat_final('Did you drink water?',F), print(F),nl.
validate_follow_up(eat,'Was it tasty?'):- follow_up_eat_final('Was it tasty?',F), print(F),nl.
/* TALK */
validate_follow_up(talk,strangers):- follow_up_talk_final(strangers,F), print(F),nl.
/* STUDY */
validate_follow_up(study,'Was it easy?'):- follow_up_study_final('Was it easy?',F), print(F),nl.
validate_follow_up(study,'Do you need help with homework?'):- follow_up_study_final('Do you need help with homework?',F), print(F), nl.
/* DANCE */
validate_follow_up(dance,'Did you bump into anybody?') :- follow_up_dance_final('Did you bump into anybody?',F), print(F), nl.
validate_follow_up(dance,'Are you able to remember the steps?') :- follow_up_dance_final('Are you able to remember the steps?',F), print(F), nl.
/* EXERCISE */
validate_follow_up(exercise,'Did you fall down?') :- follow_up_exercise_final('Did you fall down?',F), print(F),nl.
/* SEE */
validate_follow_up(see,'Did you see anything you do not know about?') :- follow_up_see_final('Did you see anything you do not know about?',F), print(F),nl.
validate_follow_up(see,'Was there any interesting objects?') :- follow_up_see_final('Was there any interesting objects?',F), print(F),nl.

/* Encourage/ conclusion message bank -> segments to if a query is answered yes or no then give relevant message. */
/* General structure : returns a relevant message, depending on the KB answered_yes_additional and answered_no, as variable F to the calling method for printing. */
follow_up_play_final('Did you injure yourself?',F):-
answered_yes_additional('Did you injure yourself?'), F='Are you okay? Be more careful while playing!';
answered_no('Did you injure yourself?'), F='Do still becareful!'.

follow_up_play_final('Did you enjoy yourself?',F) :-
answered_yes_additional('Did you enjoy yourself?'), F='I am glad you had fun!';
answered_no('Did you enjoy yourself?'), F='Maybe you should play around with friends!'.
follow_up_eat_final('Did you wash your hands?', F):-
answered_yes_additional('Did you wash your hands?'), F='Good job on washing your hands!';
answered_no('Did you wash your hands?'), F='Remember to wash if not you will be sick!'.

follow_up_eat_final('Did you drink water?',F):-
answered_yes_additional('Did you drink water?'), F='Nicely done!';
answered_no('Did you drink water?'), F='Water is important for your health, remember to drink next time!'.

follow_up_eat_final('Was it tasty?',F):-
answered_yes_additional('Was it tasty?'), F='I am glad for you';
answered_no('Was it tasty?'), F='Okay, do not worry I will cook something delicious for you later!'.

follow_up_talk_final(strangers, F):-
answered_yes(strangers), F='Do not talk to strangers next time!';
answered_no(strangers), F='Good job! Becareful about strangers'.

follow_up_study_final('Was it easy?',F):-
answered_yes_additional('Was it easy?'), F='Do not get too confident!';
answered_no('Was it easy?'), F='Do not worry, I will revise with you!'.

follow_up_study_final('Do you need help with homework?', F):-
answered_yes_additional('Do you need help with homework?'), F='Okay, I will help you after dinner!';
answered_no('Do you need help with homework?'), F='Okay, find me if you need any help!'.

follow_up_dance_final('Did you bump into anybody?', F) :-
answered_yes_additional('Did you bump into anybody?'), F='Becareful next time! Hopefully you said sorry!';
answered_no('Did you bump into anybody?'), F='Do becareful still!'.

follow_up_dance_final('Are you able to remember the steps?',F) :-
answered_yes_additional('Are you able to remember the steps?'), F='Nice! Dance for me next time.';
answered_no('Are you able to remember the steps?'), F='Its okay, practice makes perfect!'.

follow_up_exercise_final('Did you fall down?',F):-
answered_yes_additional('Did you fall down?'), F='Oh my god! Are you okay? Let me take a look at you!';                              
answered_no('Did you fall down?'), F='Good good, remember to becareful always!'.

follow_up_see_final('Did you see anything you do not know about?',F):-
answered_yes_additional('Did you see anything you do not know about?'), F='Remember to ask the teacher if you do not know anything!';
answered_no('Did you see anything you do not know about?'), F='Nice, that means you can teach me next time!'.

follow_up_see_final('Was there any interesting objects?',F):-
answered_yes_additional('Was there any interesting objects?'), F='Nice, maybe you can share it during dinner later!';
answered_no('Was there any interesting objects?'), F='That is okay, you will find something interesting soon enough!'.

/* Asked all */
asked(nothing).
/* Answered yes subjects, activities, conclusion messages*/
answered_yes(nothing).
/* Answered no subjects, activities, conclusion messages */
answered_no(nothing).
/* Answered yes follow_up messages */
answered_yes_additional(nothing).
