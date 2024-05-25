Этот репозиторий посвящен сисетмам моделей цифровых двойников


# 1. Предлодение по системе

Система будет предстовалять следующую иерархию:
1. Карта - это набор тайлов, которые в нее включены.Карта является валидной, если валидны все ее тайлы.
2. Тайл - это нобор структур данных, описывающих путевое развитие ЖД. Тайл является валидным, если все составы, находящиеся в тайле вылидны
3. Состав - это сущность, которая передвигается по путевому графу тайла. Состав валиден, когда он привязан хоть к 1 из структур тайла. (Условием является наличие какой-либо структуры (сегмент/линия), на какоторой в данный момент времени находится состав)

Датчики, которые будут имитировать движение будут находится в докер контейнерах и постоянно передовать данные для составов

![model_arch.png](images/model_arch.png)