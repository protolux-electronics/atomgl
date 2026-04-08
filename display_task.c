/*
 * This file is part of AtomGL.
 *
 * Copyright 2026 Davide Bettio <davide@uninstall.it>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "display_task.h"

#include <memory.h>
#include <utils.h>

NativeHandlerResult display_driver_consume_mailbox(Context *ctx)
{
    struct DisplayTaskArgs *args = ctx->platform_data;

    MailboxMessage *mbox_msg = mailbox_take_message(&ctx->mailbox);
    Message *msg = CONTAINER_OF(mbox_msg, Message, base);

    // Non-blocking enqueue; drop oldest on overflow.
    if (xQueueSend(args->messages_queue, &msg, 0) != pdTRUE) {

        Message *old = NULL;
        if (xQueueReceive(args->messages_queue, &old, 0) == pdTRUE && old) {
            BEGIN_WITH_STACK_HEAP(1, temp_heap);
            mailbox_message_dispose(&old->base, &temp_heap);
            END_WITH_STACK_HEAP(temp_heap, ctx->global);
        }

        if (xQueueSend(args->messages_queue, &msg, 0) != pdTRUE) {
            BEGIN_WITH_STACK_HEAP(1, temp_heap2);
            mailbox_message_dispose(&msg->base, &temp_heap2);
            END_WITH_STACK_HEAP(temp_heap2, ctx->global);
        }
    }

    return NativeContinue;
}

void display_process_messages(void *arg)
{
    struct DisplayTaskArgs *args = arg;

    while (true) {
        Message *message;
        xQueueReceive(args->messages_queue, &message, portMAX_DELAY);
        args->process_message_fn(message, args->ctx);

        BEGIN_WITH_STACK_HEAP(1, temp_heap);
        mailbox_message_dispose(&message->base, &temp_heap);
        END_WITH_STACK_HEAP(temp_heap, args->ctx->global);
    }
}
