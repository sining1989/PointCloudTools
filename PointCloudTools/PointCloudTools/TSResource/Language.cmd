::                         �������������ļ�

echo off

cd ..

echo.
echo.
echo =================================================================
echo =                     ���������������                          =
echo =================================================================
echo.
echo.

del lang.pro
echo.
echo.

echo 1�����»��������Թ����ļ�lang.pro
qmake -project -o lang.pro
echo.
echo.

echo 2���༭���Թ����ļ�lang.pro
echo CODECFORTR = System >> lang.pro
echo CODECFORSRC = System >> lang.pro
echo.
echo.

echo 3���������Է���ts�ļ�
pause
lupdate lang.pro
echo.
echo.

echo 4���༭���Է���ts�ļ�
pause 
linguist TSResource/lang_en.ts TSResource/lang_zh_CN.ts 
echo.
echo.

echo 5���������Է���ts�ļ�����qm�ļ�
pause
lrelease lang.pro
echo.
echo.

echo 6������qm�ļ���languageĿ¼��
pause
cd TSResource
move *.qm  ../Resources/qm
cd ..
echo.
echo.

del lang.pro

echo �������
echo.
echo.

pause

