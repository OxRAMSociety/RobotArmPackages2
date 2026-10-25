import asyncio
import time


def do_cuda_stuff():
    while True:
        print("Doing CUDA stuff...")
        time.sleep(3)


async def do_cuda_stuff_async():
    do_cuda_stuff()


async def regular_tasks_async():
    while True:
        await asyncio.sleep(1)
        print("Doing regular tasks...")


if __name__ == "__main__":
    task_1 = asyncio.create_task(
        regular_tasks_async()
        )
    task_2 = asyncio.create_task(
        do_cuda_stuff_async()
        )
    asyncio.run(asyncio.gather(task_1, task_2))