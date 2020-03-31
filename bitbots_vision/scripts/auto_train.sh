while read p; do
	./auto_train.py "$p"
done <models
