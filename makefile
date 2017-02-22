all:
	javac *.java -Xlint

client:
	appletviewer Client.java -J-Djava.security.policy=Client.policy

server:
	java ControlServer

clean:
	rm *.class
