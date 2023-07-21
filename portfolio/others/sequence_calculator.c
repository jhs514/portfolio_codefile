// 제목: 수열 계산기
#include <stdio.h>
#include <math.h>

int str_compare(char*a,char*b) {
	for (int i=0;a[i];i++) {
		if (a[i]!=b[i]){
			return 0;
		}
	}
	return 1;
}
void main() {
	char kind[100];
	int a1,n;
	printf("수열의 종류를 입력하시오.(등차수열, 등비수열)\n");
	scanf("%s",kind);
	if (str_compare(kind,"등차수열")) {
		int d;
		printf("첫째항을 입력하시오.\n");
		scanf("%d",&a1);
		printf("공차를 입력하시오.\n");
		scanf("%d",&d);
		if (a1-d>=0) {
			printf("일반항 : %d*n+%d\n",d,a1-d);
		} else if (a1-d<0) {
			printf("일반항 : %d*n%d\n",d,a1-d);
		}
		while (n!=0) {
			printf("구하고 싶은 항의 번호를 입력하시오.(종료하고 싶으면 0을 입력)\n");
			scanf("%d",&n);
			printf("%d",d*n+a1-d);
		}
	} else if (str_compare(kind,"등비수열")) {
		int r;
		printf("첫째항을 입력하시오.\n");
		scanf("%d",&a1);
		printf("공비를 입력하시오.\n");
		scanf("%d",&r);
		printf("일반항 : %d*%d**(n-1)\n",a1,r);
		while (n!=0) {
			printf("구하고 싶은 항의 번호를 입력하시오.(종료하고 싶으면 0을 입력)\n");
			scanf("%d",&n);
			int R = pow(r,n-1);
			printf("%d",a1*R);
		}
	} else {
		printf("ERROR");
	}
}